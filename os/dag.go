package main 

/*
 * This file implements DAG or dynamic task graph (DTG), involving maintaining
 * the DAG and using DAG for application forecasting. 
 * 
 */

import (
	"strings"
	"encoding/json"
	"fmt"
	"sync"
	"github.com/deckarep/golang-set"
	"sort"
	"math"
	"github.com/llgcode/draw2d/draw2dimg"
	"github.com/llgcode/draw2d/draw2dkit"
	"image"
	"image/color"
	"image/png"
	"os"
	"bufio"
	"encoding/gob"
)

var CFG_MAXNODE int = 100 
var CFG_MERGEP_REDICTION_RANGE float64 = 5.0 // original 10.0
var MinimalDepth int = 0


const (
	ROOT = 0
	NEW_TASK  = 1
	CODEBLOCK = 2 
	CANCEL_TASK = 3
	SYNC = 4
)

type DAGNode struct {
	Name 		string 
	NameItems	[]string 
	PureName	string // remove ids
	NodeType 	int 
	CancelName string 

	MetaInfo 	string

	ScopeName	string

	Location	[3]float64
	LocationUnknown bool
	Duration  	float64 
	Sensors		[]string


	Parents []*DAGNode
	Children []*DAGNode

	// following are used for prediction and speculation 
	Prob		float64

	Ordering	int 

	
}

type DAGNodeString struct {
	Name 		string 
	NameItems	[]string 
	PureName	string // remove ids
	NodeType 	int 
	CancelName string

	MetaInfo 	string

	ScopeName	string

	Location	[3]float64
	LocationUnknown bool
	Duration  	float64 
	Sensors		[]string


	Parents []string
	Children []string

	// following are used for prediction and speculation 
	Prob		float64

	Ordering	int 
}


type Subgraph struct {
	dag 			*DAG
	startNode 		*DAGNode
	terminateNode   *DAGNode
	score			float64
}

type VirtualNode struct {
	NodeType	int

	Location 		[3]float64
	Sensors		[]string
	Duration  	float64 

	Depth 		int 

	Prob		float64

	CancelOtherTask				bool
	CancelOtherTaskMask			string
	CancelOtherTaskRadiusProb 	[20]float64 // may have different distribution
	CancelOtherTaskRadius       float64
	
	// add dependency later [TODO]
}

func (vn *VirtualNode) GetLocation() [3]float64 {
	return vn.Location  
}

func (vn *VirtualNode) GetType() int {
	return PREDICTION_REQUEST 
}

type Prediction struct {
	Nodes		[]*VirtualNode
	TS 			int64

}


type DAG struct {
	Root		*DAGNode
	Name2Node	map[string]*DAGNode

	dagLock		sync.Mutex
	nodecounter	int

	Frontiers   map[*DAGNode]*Prediction
	GlobalPrediction *Prediction

	historyDags []*DAG

	debuglog 	bool
}

type DAGString struct {
	Name2Node	map[string]*DAGNodeString
}


func (dagstr *DAGString) toDAG() *DAG{
	dag := new(DAG)

	dag.Name2Node = make(map[string]*DAGNode)

	for k,v := range dagstr.Name2Node {
		//fmt.Println(k)
		node := new(DAGNode)

		node.Name = v.Name 
		node.NameItems = v.NameItems
		node.PureName = v.PureName
		node.NodeType = v.NodeType
		node.CancelName = v.CancelName

		node.MetaInfo = v.MetaInfo

		node.ScopeName = v.ScopeName

		node.Location = v.Location
		node.LocationUnknown = v.LocationUnknown
		node.Duration = v.Duration
		node.Sensors = v.Sensors

		node.Ordering = v.Ordering

		dag.Name2Node[k] = node

	}

	for k,v := range dagstr.Name2Node {

		for _, name := range v.Parents {
			if name == "" {
				name = "root"
			}
			dag.Name2Node[k].Parents = append(dag.Name2Node[k].Parents, dag.Name2Node[name])
		}

		for _, name := range v.Children {
			if name == "" {
				name = "root"
			}
			dag.Name2Node[k].Children = append(dag.Name2Node[k].Children, dag.Name2Node[name])
		}

	}

	return dag 
}

func (dag *DAG) toDAGString() *DAGString{
	dagstr := new(DAGString)

	dagstr.Name2Node = make(map[string]*DAGNodeString)

	for k, v := range dag.Name2Node {
		node := new(DAGNodeString)

		node.Name = v.Name 
		node.NameItems = v.NameItems
		node.PureName = v.PureName
		node.NodeType = v.NodeType
		node.CancelName = v.CancelName

		node.MetaInfo = v.MetaInfo

		node.ScopeName = v.ScopeName

		node.Location = v.Location
		node.LocationUnknown = v.LocationUnknown
		node.Duration = v.Duration
		node.Sensors = v.Sensors

		node.Ordering = v.Ordering

		for _, pn := range v.Parents {
			node.Parents = append(node.Parents, pn.Name)
		}

		for _, pn := range v.Children {
			node.Children = append(node.Children, pn.Name)
		}
		
		dagstr.Name2Node[k] = node

	}

	return dagstr
}



func IsSameNode(n1,n2 *DAGNode) bool {
	if n1 == nil && n2 == nil {
		return true 
	}

	if n1 == nil || n2 == nil {
		return false 
	}

	if n1.NodeType != n2.NodeType {
		return false 
	}

	if n1.PureName != n2.PureName {
		return false 
	}

	return true
}

func (sg *Subgraph) Size() int {
	//fmt.Println("trace", sg)
	if sg.startNode == sg.terminateNode {
		return 1
	}

	if len(sg.startNode.Parents) > 1 {
		asc := sg.dag.findCommonAscendant(sg.startNode)

		if asc != sg.terminateNode {
			newsg1 := Subgraph{sg.dag, sg.startNode, asc, 0.0}
			newsg2 := Subgraph{sg.dag, asc, sg.terminateNode, 0.0}
			//fmt.Println("-- mark1")
			s1 := newsg1.Size()
			//fmt.Println("-- mark2")
			s2 := newsg2.Size()

			return s1+s2-1
		}

	} 


	var ret int = 1
	//fmt.Println("-- ",sg.startNode.Parents)
	for _,nextnode := range sg.startNode.Parents {
		newsg := Subgraph{sg.dag, nextnode, sg.terminateNode, 0.0}
		//fmt.Println("-- mark3")
		ret += newsg.Size()
	}

	return ret - (len(sg.startNode.Parents)-1)
}

type nodeCallback func(*DAGNode)

func (sg *Subgraph) _ForEach(fn nodeCallback) {
	if sg.startNode == sg.terminateNode {
		fn(sg.startNode)
		return 
	}

	if len(sg.startNode.Parents) > 1 {
		asc := sg.dag.findCommonAscendant(sg.startNode)

		if asc != sg.terminateNode {
			newsg1 := Subgraph{sg.dag, sg.startNode, asc, 0.0}
			newsg2 := Subgraph{sg.dag, asc, sg.terminateNode, 0.0}

			newsg1._ForEach(fn)
			newsg2._ForEach(fn)
			return 
		}

	} 

	fn(sg.startNode)

	for _,nextnode := range sg.startNode.Parents {
		newsg := Subgraph{sg.dag, nextnode, sg.terminateNode, 0.0}

		newsg._ForEach(fn)
	}
}


func (sg *Subgraph) ForEach(fn nodeCallback) {

	nodes := make(map[*DAGNode]bool)

	f2 := func(node *DAGNode) {
		nodes[node] = true 
	}

	sg._ForEach(f2)


	for k,_ := range nodes {
		fn(k)
	}
}


func (sg *Subgraph) Draw(name string) {
	var locs [][3]float64


	fn := func(node *DAGNode) {
		if node.NodeType == CODEBLOCK {
			locs = append(locs, node.Location)
		}
	}

	sg.ForEach(fn)

	DrawLocations(locs, name)

}

func (sg *Subgraph) Locations() [][3]float64 {
	var locs [][3]float64


	fn := func(node *DAGNode) {
		if node.NodeType == CODEBLOCK {
			locs = append(locs, node.Location)
		}
	}

	sg.ForEach(fn)

	return locs 
}


func NewPredictionFromSubGraph(g Subgraph, trans Transformation, graphProb float64, timeDecay float64, maxNode int ) *Prediction{
	pred := new(Prediction)

	//baseOrdering := g.startNode.Ordering

	nodemap := make(map[*DAGNode]int) // existing and depth


	nodemap[g.startNode] = 0

	var queue []*DAGNode

	queue = append(queue, g.startNode)
	var counter int = 0 

	for {
		if len(queue) == 0 {
			break
		}

		counter += 1 

		if counter > maxNode {
			break
		}

		currentnode := queue[0]
		queue = queue[1:len(queue)]

		var vnode VirtualNode
		var isVnodeAdded bool = false 
		if currentnode.NodeType == CODEBLOCK {
			v := nodemap[currentnode]
			

			vnode.NodeType = CODEBLOCK
			vnode.Depth = v

			for _, sensor := range currentnode.Sensors {
				vnode.Sensors = append(vnode.Sensors, sensor)
			}

			if trans.NoLocation == false {
				vnode.Location = ApplyTransformation(currentnode.Location, trans)
			} else {
				vnode.Location = currentnode.Location 
			}

			vnode.Prob = graphProb * math.Pow(timeDecay, float64(v))

			if v >= 1 {
				// add virtual node
				// we should avoid the 'first' codeblock
				// as it might have already filed the 
				// request 
				// if currentnode.LocationUnknown == true {
				// 	panic("LocationUnknown")
				// }
				if trans.OnlyBias == false && currentnode.LocationUnknown == false{
					pred.Nodes = append(pred.Nodes, &vnode)
					isVnodeAdded = true 
				}

			}
			//nodemap[currentnode] = nodemap[currentnode] + 1
		} 

		if currentnode.NodeType == NEW_TASK {
			//nodemap[currentnode] = nodemap[currentnode] + 1
		}

		if currentnode.NodeType == SYNC {
			//nodemap[currentnode] = nodemap[currentnode] + 1	
		}


		var hasCancelTask bool = false 
		var maxCancelDistance float64 = 0.0

		for _, nextnode := range currentnode.Children {
			if _,ok := nodemap[nextnode]; ok == false {
				nodemap[nextnode] = nodemap[currentnode]
				queue = append(queue, nextnode)
			}

			// TODO dealwith cancel task here?
			if vnode.NodeType == CODEBLOCK {
				nodemap[nextnode] = nodemap[currentnode] + 1
				// TODO TODO TODO  ???
				// need to know the cancelled tasks' location!!!!
				// Update CancelOtherTaskRadiusProb

				if nextnode.NodeType == CANCEL_TASK {
					if _, ok := g.dag.Name2Node[nextnode.CancelName]; ok {
						//fmt.Println("############", nextnode.CancelName, g.dag.Name2Node[nextnode.CancelName].LocationUnknown)
						if g.dag.Name2Node[nextnode.CancelName].LocationUnknown == false {
							d := distance(g.dag.Name2Node[nextnode.CancelName].Location, currentnode.Location)
							if d > maxCancelDistance {
								maxCancelDistance = d 
								hasCancelTask = true 
							}


							ind := int(d/20.0)
							if ind >= 20 {
								ind = 19 
							}

							vnode.CancelOtherTaskRadiusProb[ind]  += 1

						}
					}
				}
			}
		}


		if vnode.NodeType == CODEBLOCK && hasCancelTask == true {

			vnode.CancelOtherTask = true
			vnode.CancelOtherTaskRadius = maxCancelDistance
			vnode.Depth = nodemap[currentnode] + 1
			vnode.Prob = graphProb * math.Pow(timeDecay, float64(vnode.Depth))
			if isVnodeAdded == false {
				pred.Nodes = append(pred.Nodes, &vnode)
			}
			
		}



	}
	return pred
}

type MergeFunc func(a,b float64) float64 


// when the distance between two nodes is less than 'threshold', merge them
func MergePredictions(preds []*Prediction, threshold float64, mergeFunc MergeFunc) *Prediction {
	pred := new(Prediction)
	pred.TS = makeTimestamp()

	// TODO optimize with RTree


	var lastPredictionEndPtr int = 0 

	for i:=0; i< len(preds) ; i++ {

		for j:=0; j<len(preds[i].Nodes); j++ {

			var isExist bool = false 
			var best_d float64 = threshold
			var best_vnode *VirtualNode

			for k:=0; k<lastPredictionEndPtr; k++ {
				d := distance(pred.Nodes[k].Location, preds[i].Nodes[j].Location)
				if d < best_d {
					isExist = true 
					best_vnode = pred.Nodes[k]
				}

			}

			if isExist {
				best_vnode.Prob = mergeFunc(best_vnode.Prob, preds[i].Nodes[j].Prob)
				best_vnode.CancelOtherTaskRadius = (best_vnode.CancelOtherTaskRadius + preds[i].Nodes[j].CancelOtherTaskRadius) / 2.0

				best_vnode.CancelOtherTaskRadiusProb = arrAdd20(best_vnode.CancelOtherTaskRadiusProb, preds[i].Nodes[j].CancelOtherTaskRadiusProb)
			} else {
				pred.Nodes = append(pred.Nodes, preds[i].Nodes[j])
			}
		}

		lastPredictionEndPtr = len(pred.Nodes)
	}
	return pred
}

func (pred *Prediction) Normalization() {
	var maxprob float64 = 0.0 

	for _, node := range pred.Nodes{
		if node.Prob > maxprob {
			maxprob = node.Prob 
		}
	}

	for _, node := range pred.Nodes{
		node.Prob = node.Prob/(maxprob+0.001)

		node.CancelOtherTaskRadius = arrMedian20(node.CancelOtherTaskRadiusProb) * 20.0
	}

}



func MakeDAG(history []*DAG) *DAG {
	dag := new(DAG)
	dag.Name2Node = make(map[string]*DAGNode)

	dag.Name2Node["root"] = new(DAGNode)
	dag.Name2Node["root"].Name = "root"

	dag.Frontiers = make(map[*DAGNode]*Prediction)

	dag.historyDags = history

	return dag 
}

// debug
var AddDependencyCounter int = 0 



func (dag *DAG) AddDeferredLocation(nodename string, loc [3]float64) {
	dag.dagLock.Lock()
	defer dag.dagLock.Unlock()

	//fmt.Println(nodename, loc)

	dag.Name2Node[nodename].Location = loc 
	dag.Name2Node[nodename].LocationUnknown = false

	//dag.updatePrediction()

}


func (dag *DAG) AddDependency(nodename string, dependencies []string, scope string, metainfo string) {
	dag.dagLock.Lock()
	defer dag.dagLock.Unlock()

	
	var currentnode *DAGNode
	var newnode bool = true 
	var canrepeat bool = false 

	if n,ok := dag.Name2Node[nodename]; ok {
		currentnode = n
		newnode = false 
	} else {
		currentnode = new(DAGNode)
		dag.Name2Node[nodename] = currentnode
	}

	currentnode.Name = nodename
	currentnode.NameItems = strings.Split(nodename, "_")
	currentnode.LocationUnknown = true 

	currentnode.MetaInfo = metainfo
	currentnode.ScopeName = scope


	if strings.HasPrefix(nodename,"syncBlock") {
		currentnode.NodeType = SYNC
		if strings.HasPrefix(nodename, "syncBlockSIMD") {
			canrepeat = true 
		}

	} else if strings.HasPrefix(nodename,"cancel") {
		currentnode.NodeType = CANCEL_TASK
	} else if strings.HasPrefix(nodename,"codeBlock") {
		currentnode.NodeType = CODEBLOCK
	} else {
		currentnode.NodeType = NEW_TASK
	}

	if currentnode.NodeType == SYNC {
		currentnode.PureName = currentnode.NameItems[0]
	}

	if currentnode.NodeType == CODEBLOCK {
		currentnode.PureName = currentnode.NameItems[len(currentnode.NameItems)-2]
	}

	if currentnode.NodeType == NEW_TASK {
		currentnode.PureName = ""
		for i:=0; i<len(currentnode.NameItems)-1;i++{
			currentnode.PureName += currentnode.NameItems[i]
		}

	}

	if currentnode.NodeType == CANCEL_TASK {
		currentnode.PureName = "Cancel"

		// update the cancel name

		currentnode.CancelName = strings.TrimPrefix(nodename, "cancel_")


		if node,ok := dag.Name2Node[currentnode.CancelName]; ok {
			// TODO recursivly delete all the nodes
			// Now, just remove the first layer 
			for _, childnode := range node.Children {
				if _,ok2 := dag.Frontiers[childnode]; ok2 {
					delete(dag.Frontiers, childnode)
				}	
			}

			if _,ok2 := dag.Frontiers[node]; ok2 {
				delete(dag.Frontiers, node)
			}
		}

	}



	if newnode || canrepeat{
		if currentnode.NodeType != CANCEL_TASK {
			dag.Frontiers[currentnode] = nil
		}

		var min_ordering int = math.MaxInt32
		for _, dep := range dependencies {
			pn := dag.Name2Node[dep]

			if pn.Ordering < min_ordering {
				min_ordering = pn.Ordering
			}

			pn.Children = append(pn.Children, currentnode)
			currentnode.Parents = append(currentnode.Parents, pn)

			if _,ok := dag.Frontiers[pn]; ok {
				delete(dag.Frontiers, pn)
			}
		}
		//if canrepeat == false {
		currentnode.Ordering = min_ordering + 1
		//} else {
		//	currentnode.Ordering = min_ordering
		//}

	}

	if strings.HasPrefix(metainfo, "\"nul") == false {
		//fmt.Println(metainfo)
		metadict := make(map[string]interface{})

		if err:=json.Unmarshal([]byte(metainfo[1:len(metainfo)-1]), &metadict); err!=nil {
			panic(err)
		} else {
			//fmt.Println(metadict["firstloc"])
			//iloc := metadict["firstloc"].([]interface{})
			//currentnode.Location = [3]float64{iloc[0].(float64),iloc[1].(float64),iloc[2].(float64) }
			//currentnode.LocationUnknown = false
			//fmt.Println(currentnode.Location)
		}
	}

	dag.nodecounter = dag.nodecounter + 1 


	//dag.updatePrediction()


	return
}

func (dag *DAG) LogAction() {

}

func (dag *DAG) getPredictionForNode(name string) *Prediction {
	dag.dagLock.Lock()
	defer dag.dagLock.Unlock()

	//dag.updatePrediction()

	if pred,ok:= dag.Frontiers[dag.Name2Node[name]];ok {
		return pred
	}

	return nil 
}

func (dag *DAG) getGlobalPrediction() *Prediction {
	dag.dagLock.Lock()
	defer dag.dagLock.Unlock()

	dag.updatePrediction()

	return dag.GlobalPrediction
}




func (dag *DAG) updatePrediction() {
	//test

	// update prediction 

	var preds []*Prediction

	tnow := makeTimestamp()

	for node, pred := range dag.Frontiers {
		// update all frontier 
		// update 'old' predictions which are more than 20 seconds old
		if pred == nil || (pred != nil && pred.TS < tnow - 20000 ) {
			// if node.NodeType == NEW_TASK && node.LocationUnknown == false {
			// 	dag.Frontiers[node] = dag.match(node, append([]*DAG{dag}, dag.historyDags...))
			// }

			if (node.NodeType == CODEBLOCK) && node.LocationUnknown == false {
				dag.Frontiers[node] = dag.match(node, append([]*DAG{dag}, dag.historyDags...))
				pred = dag.Frontiers[node] 
			}

		}

		//fmt.Println(node)
		if quiet_log == false {
			fmt.Println("Frontier", node.Name)//, len(dag.Frontiers[node].Nodes))
		}
		if pred != nil {
			if len(dag.Frontiers[node].Nodes)==0 {
				delete(dag.Frontiers, node)
			} else {

				preds = append(preds, dag.Frontiers[node])
			}
		}
	}


	globalPrediction := MergePredictions(preds, 10.0, func(a,b float64) float64 {if a>b{
			return a
		}else{
			return b
			}})


	//fmt.Println("Number of frontiers", len(dag.Frontiers), len(globalPrediction.Nodes))
	globalPrediction.DebugVisualization()


	dag.GlobalPrediction = globalPrediction

	// Following is code for debugging
	// AddDependencyCounter += 1
	AddDependencyCounter += 1

	if AddDependencyCounter > 80000 {
		PrintLog(LogDebug, fmt.Sprintln("start debugging"))

		for k,_ := range dag.Name2Node {
			PrintLog(LogDebug, fmt.Sprintln(k))
		}


		for {
			PrintLog(LogDebug, fmt.Sprintln("test node:"))

			reader := bufio.NewReader(os.Stdin)

			name, _ := reader.ReadString('\n')
			name = strings.Replace(name, "\n", "", -1)

			PrintLog(LogDebug, fmt.Sprintln("Node Name:", name))

			if _,ok := dag.Name2Node[name]; ok {
				PrintLog(LogDebug, fmt.Sprintln("try to find match"))
				_ = dag.match(dag.Name2Node[name], append([]*DAG{dag}, dag.historyDags...))

			}

		}
	}


}

// When a node has more than one parents, we would like to find the 
// common ascendant of these parents. This ascendant always exists(root)
func (dag *DAG) findCommonAscendant(node *DAGNode) *DAGNode {
	//defer printtime(makeTimestamp(), "findCommonAscendant")


	if len(node.Parents) <= 1 {
		return node 
	}


	ref_c := make(map[*DAGNode]int)

	frontiers := mapset.NewSet()
	for _,n:= range node.Parents{
		frontiers.Add(n)
		ref_c[n] = 0
	}

	for {
		stuck := true

		var nodes []*DAGNode 
		for n := range frontiers.Iter() {
			nodes = append(nodes, n.(*DAGNode))
		}

		for _, n:= range nodes{
			cur_n := n
			if ref_c[cur_n] == 0 {
				for _, pn := range cur_n.Parents {
					frontiers.Add(pn)
					if _, ok := ref_c[pn]; ok {
						ref_c[pn] = ref_c[pn]-1
					} else {
						ref_c[pn] = len(pn.Children)-1
					}
				}
				frontiers.Remove(cur_n)
				
				stuck = false 
				break // without this break --> big bug!!!
			}
		}

		

		if stuck {
			// decrease the reference counter
			// of the toppest node (ordering)

			var max_order_node *DAGNode = nil 


			for n := range frontiers.Iter() {
				node := n.(*DAGNode)

				if max_order_node == nil {
					max_order_node = node 
				} else {
					if node.Ordering > max_order_node.Ordering {
						max_order_node = node 
					}
				}
			}

			frontiers.Remove(max_order_node)
		}

		//fmt.Println(frontiers)
		//fmt.Println(ref_c)

		if frontiers.Cardinality() == 1 {
			break 
		}
	}

	return frontiers.Pop().(*DAGNode)
}


func permutations(arr []int)[][]int{
    var helper func([]int, int)
    res := [][]int{}

    helper = func(arr []int, n int){
        if n == 1{
            tmp := make([]int, len(arr))
            copy(tmp, arr)
            res = append(res, tmp)
        } else {
            for i := 0; i < n; i++{
                helper(arr, n - 1)
                if n % 2 == 1{
                    tmp := arr[i]
                    arr[i] = arr[n - 1]
                    arr[n - 1] = tmp
                } else {
                    tmp := arr[0]
                    arr[0] = arr[n - 1]
                    arr[n - 1] = tmp
                }
            }
        }
    }
    helper(arr, len(arr))
    return res
}

// several optimizations need todo
// (1) memorization
// (2) remove redudant comparision 
// NEED OPTIMIZATION !!!    n! 
func _recursive_match(subgraph1, subgraph2 Subgraph) bool{


	if IsSameNode(subgraph1.startNode, subgraph2.startNode) == false {
		return false 
	}

	if subgraph1.startNode == subgraph1.terminateNode {
		if subgraph2.startNode == subgraph2.terminateNode {
			return true 
		} else {
			return false
		}
	}

	if subgraph2.startNode == subgraph2.terminateNode {
		if subgraph1.startNode == subgraph1.terminateNode {
			return true 
		} else {
			return false
		}
	}


	if len(subgraph1.startNode.Parents) != len(subgraph2.startNode.Parents) {
		return false 
	} 


	if len(subgraph1.startNode.Parents) == 1 {
		//fmt.Println(subgraph2.startNode.Name, subgraph2.startNode.Parents[0])
		subgraph1.startNode = subgraph1.startNode.Parents[0]
		subgraph2.startNode = subgraph2.startNode.Parents[0]

		return _recursive_match(subgraph1, subgraph2)
	}

	if len(subgraph1.startNode.Parents) >1 {
	 	// n! 
		var matched [][]bool 

		N := len(subgraph1.startNode.Parents)

		var allmatch bool = true
		
		for i:=0; i<N; i++ {
			arr := []bool{}
			for j:=0; j<N;j++{

				var sub1 Subgraph
				var sub2 Subgraph

				sub1.dag = subgraph1.dag 
				sub1.startNode = subgraph1.startNode.Parents[i]
				sub1.terminateNode = subgraph1.terminateNode

				sub2.dag = subgraph2.dag 
				sub2.startNode = subgraph2.startNode.Parents[j]
				sub2.terminateNode = subgraph2.terminateNode

				b := _recursive_match(sub1,sub2)
				if b == false {
					allmatch = false
				}
				arr = append(arr, b)
			}

			matched = append(matched, arr)
		}


		

		if allmatch {
			return true
		} else {
			return false
		}

		//fmt.Println("checkallmatch failed???")

		var arr []int
		for i:=0; i<N;i++{
			arr = append(arr,i)
		}


		perm := permutations(arr)


		for k:=0;k<len(perm);k++{
			var flag = false
			for i:=0;i<N;i++{
				if matched[i][perm[k][i]] == false {
					flag = true
					break
				}
			}

			if flag == false {
				return true 
			}
		}

		return false
	}

	return true 
}



//todo rotation of a tree ...
type transMatch struct {
	l1 			int 
	l2 			int 
	distance 	float64
}

type transMatchList struct {
	ass  []transMatch
}

func (a *transMatchList) Len() int {
	return len(a.ass)
}

func (a *transMatchList) Swap(i,j int) {

	a.ass[i], a.ass[j] = a.ass[j], a.ass[i]

}

func (a *transMatchList) Less(i,j int) bool {
	return a.ass[i].distance < a.ass[j].distance 
}





func transformationMatch(subgraph1, subgraph2 Subgraph) (Transformation, float64) {
	// transform from subgraph2 to subgraph1 
	//t0 := makeTimestamp()

	var c1 float64 = 0 
	var c2 float64 = 0

	var l1 [3]float64 = [3]float64{0.0,0.0,0.0}
	var l2 [3]float64 = [3]float64{0.0,0.0,0.0}

	var locs1 [][3]float64
	var locs2 [][3]float64

	var ord1 []int 
	var ord2 []int

	var minord1 int = 10000000
	var minord2 int = 10000000


	subgraph1.ForEach(func(dn *DAGNode) {


			if dn.NodeType == CODEBLOCK || (dn.NodeType == NEW_TASK && dn.LocationUnknown == false) {
				l1 = locAdd(l1, dn.Location)
				c1 = c1 + 1.0
				locs1 = append(locs1, dn.Location)
				ord1 = append(ord1, dn.Ordering)
				if dn.Ordering < minord1 {
					minord1 = dn.Ordering
				}
			}


			
		})

	subgraph2.ForEach(func(dn *DAGNode) {
			if dn.NodeType == CODEBLOCK || (dn.NodeType == NEW_TASK && dn.LocationUnknown == false) {
				l2 = locAdd(l2, dn.Location)
				c2 = c2 + 1.0
				locs2 = append(locs2, dn.Location)
				ord2 = append(ord2, dn.Ordering)
				if dn.Ordering < minord2 {
					minord2 = dn.Ordering
				}
			}

			

		})
	var trans Transformation 

	if len(locs1) == 0 || len(locs2) == 0{
		trans.NoLocation = true 
		return trans, 1000.0
	}

	center1 := locDivide(l1, c1)
	center2 := locDivide(l2, c2)

	trans.bias = locSubtract(center1, center2)


	if len(locs1) == 1 || len(locs2) == 1 {
		trans.OnlyBias = true 
		return trans, 1000.0
	}


	for i:=0; i< len(locs1); i++ {
		locs1[i] = locSubtract(locs1[i], center1)
		ord1[i] -= minord1 
	}

	for i:=0; i<len(locs2); i++{
		locs2[i] = locSubtract(locs2[i], center2)
		ord2[i] -= minord2
	}


	// find the best angle ...
	// brute-force

	locs2_rotated := make([][3]float64, len(locs2))


	var best_min_distance float64 = 1000000000000.0 
	var best_angle float64 = 0.0

	for angle := 0.0; angle < 2.0*3.1415926; angle += 2.0*3.1415926/64.0 {

		for i:=0; i< len(locs2); i++ {
			locs2_rotated[i] = locRotate(locs2[i], angle) 
		}

		var allMatch transMatchList

		for i:=0; i< len(locs1); i++ {
			for j:=0; j< len(locs2); j++ {
				var match transMatch

				match.l1 = i 
				match.l2 = j 

				//order_diff := math.Log(math.Abs(float64(ord1[i] - ord2[j]))+1.0)
				order_diff := math.Abs(float64(ord1[i] - ord2[j])) * 10.0
				match.distance = distance(locs1[i], locs2_rotated[j]) + order_diff

				allMatch.ass = append(allMatch.ass, match)
			}
		}


		sort.Sort(&allMatch)

		map1 := make(map[int]bool)
		map2 := make(map[int]bool)

		var total_distance float64 = 0.0 
		var number_of_match int = 0 

		match_c := 0
		for _, match := range allMatch.ass {
			// ignore the last matching (could be outliers)
			if match_c > len(locs1) -2 || match_c > len(locs2) - 2{
				break
			}

			l1 := match.l1 
			l2 := match.l2 

			if _,ok := map1[l1]; ok {
				continue 
			}

			if _,ok := map2[l2]; ok {
				continue 
			}

			map1[l1] = true 
			map2[l2] = true 

			if match.distance < 10.0 {
				number_of_match += 1 
			} else {
				//total_distance += match.distance
			}

			total_distance += match.distance
			match_c += 1
			// sanity check
			// if ind < 10 {
			// 	fmt.Println(ind, match.distance)
			// }

		}

		if total_distance - float64(number_of_match)*10.0 < best_min_distance {
			best_min_distance = total_distance - float64(number_of_match)*10.0
			best_angle = angle 
		}

		//fmt.Println("angle", angle, total_distance)
	}

	//fmt.Println("best angle", best_angle, best_min_distance)



	trans.center = center2
	trans.angle = best_angle
	trans.scale = 1.0

	return trans, best_min_distance
}


func rotationHistogramSmooth(in []float64, dim int) []float64 {
	o := make([]float64, dim)

	for i :=0; i< dim; i++ {
		v := 0.0
		for j := i - 1 ; j < i+2; j++ {
			ind := j 
			if ind >= dim {
				ind = ind - dim 
			}

			if ind < 0 {
				ind = ind + dim 
			}
			
			v += in[ind]
		}

		o[i] = v/3.0
	}

	return o
}

func rotationHistogramSmoothN(in []float64, dim int, it int) []float64 {
	o := in
	for i:=0; i<it; i++ {
		o = rotationHistogramSmooth(o, dim)
	}

	return o
}



func rotationHistogram(locs [][3]float64, ord []int, ordthreshold int,  dim int) ([]float64, []float64) {
	highord := make([]float64, dim)
	loword := make([]float64, dim)

	highdist := make([]float64, dim)
	lowdist := make([]float64, dim)


	for ind, loc := range locs {
		order := ord[ind]

		angle := int((math.Atan2(loc[1], loc[0]) / (2.0*math.Pi) + 0.5) * float64(dim)) % dim
		
		if order >= ordthreshold {
			highord[angle] += 1
			highdist[angle] += distance(loc, [3]float64{0.0, 0.0, 0.0})
		}

		if order <= ordthreshold {
			loword[angle] += 1
			lowdist[angle] += distance(loc, [3]float64{0.0, 0.0, 0.0})
		}
	}

	for i:=0; i<dim; i++ {
		if highord[i] > 0.0 {
			highord[i] = highdist[i]/highord[i]
		} 

		if loword[i] > 0.0 {
			loword[i] = lowdist[i]/loword[i]
		}
	}

	return rotationHistogramSmoothN(loword, dim, 3), rotationHistogramSmoothN(highord, dim, 3)
}



func rotationHistogramMatch(template []float64, target []float64, dim int) int {
	max1 := 0
	max2 := 0 

	for i :=0; i< dim; i++ {
		if template[i] > template[max1] {
			max1 = i 
		}

		if target[i] > target[max2] {
			max2 = i 
		}
	}


	if max2 >= max1 {
		return max2 - max1
	} else {
		return max2 + dim - max1 
	}
}

func rotationHistogramMatchExact(template1 []float64, template2 []float64, target1 []float64, target2 []float64, dim int) int {
	best_d := 10000000.0 
	var best_d_angle int = 0

	for i :=0; i< dim; i++ {
		d := 0.0 
		for j:=0; j<dim; j ++ {
			ind1 := j
			ind2 := (j + i) % dim

			d += math.Abs(template1[ind1] - target1[ind2])
			d += math.Abs(template2[ind1] - target2[ind2])
		}

		if d < best_d {
			best_d = d
			best_d_angle = i
		}
	}


	return best_d_angle
}

func angleDiffInt(a int, b int, dim int) int {
	if a < b {
		a,b = b,a
	}
	
	d1 := a - b 
	d2 := dim - d1

	if d1 > d2 {
		return d2
	} else {
		return d1
	}

}




func fastTransformationMatch(subgraph1, subgraph2 Subgraph) (Transformation, float64) {
	// transform from subgraph2 to subgraph1 
	//t0 := makeTimestamp()

	var c1 float64 = 0 
	var c2 float64 = 0

	var l1 [3]float64 = [3]float64{0.0,0.0,0.0}
	var l2 [3]float64 = [3]float64{0.0,0.0,0.0}

	var locs1 [][3]float64
	var locs2 [][3]float64

	var ord1 []int 
	var ord2 []int

	var minord1 int = 10000000
	var minord2 int = 10000000


	subgraph1.ForEach(func(dn *DAGNode) {


			if dn.NodeType == CODEBLOCK || (dn.NodeType == NEW_TASK && dn.LocationUnknown == false) {
				l1 = locAdd(l1, dn.Location)
				c1 = c1 + 1.0
				locs1 = append(locs1, dn.Location)
				ord1 = append(ord1, dn.Ordering)
				if dn.Ordering < minord1 {
					minord1 = dn.Ordering
				}
			}


			
		})

	subgraph2.ForEach(func(dn *DAGNode) {
			if dn.NodeType == CODEBLOCK || (dn.NodeType == NEW_TASK && dn.LocationUnknown == false) {
				l2 = locAdd(l2, dn.Location)
				c2 = c2 + 1.0
				locs2 = append(locs2, dn.Location)
				ord2 = append(ord2, dn.Ordering)
				if dn.Ordering < minord2 {
					minord2 = dn.Ordering
				}
			}

			

		})
	var trans Transformation 

	if len(locs1) == 0 || len(locs2) == 0{
		trans.NoLocation = true 
		return trans, 1000.0
	}

	center1 := locDivide(l1, c1)
	center2 := locDivide(l2, c2)

	trans.bias = locSubtract(center1, center2)

	if len(locs1) == 1 || len(locs2) == 1 {
		trans.OnlyBias = true 
		return trans, 1000.0
	}

	maxord := 0 

	for i:=0; i< len(locs1); i++ {
		locs1[i] = locSubtract(locs1[i], center1)
		ord1[i] -= minord1 

		if ord1[i] > maxord {
			maxord = ord1[i]
		}
	}

	for i:=0; i<len(locs2); i++{
		locs2[i] = locSubtract(locs2[i], center2)
		ord2[i] -= minord2
	}


	f1_l, f1_h := rotationHistogram(locs1, ord1, maxord/2, 64)
	f2_l, f2_h := rotationHistogram(locs2, ord2, maxord/2, 64)

	//_, f1_h := rotationHistogram(locs1, ord1, maxord/2, 64)
	//_, f2_h := rotationHistogram(locs2, ord2, maxord/2, 64)


	a_h := rotationHistogramMatch(f1_h, f2_h, 64)
	a_exact := rotationHistogramMatchExact(f1_h, f1_l, f2_h, f2_l, 64)

	//a_l := rotationHistogramMatch(f1_l, f2_l, 64)

	if quiet_log == false {
		fmt.Println("maxMatch", a_h, "exactMatch", a_exact)
	}
	// find the best angle ...
	// brute-force

	locs2_rotated := make([][3]float64, len(locs2))


	var best_min_distance float64 = 1000000000000.0 
	var best_angle float64 = 0.0


	angle := float64(63 - a_exact) * 2.0*3.1415926/64.0
	//angle := float64(a_exact) * 2.0*3.1415926/64.0
	//for angle := 0.0; angle < 2.0*3.1415926; angle += 2.0*3.1415926/64.0 {
 	
		for i:=0; i< len(locs2); i++ {
			locs2_rotated[i] = locRotate(locs2[i], angle) 
		}

		var allMatch transMatchList

		for i:=0; i< len(locs1); i++ {
			for j:=0; j< len(locs2); j++ {
				var match transMatch

				match.l1 = i 
				match.l2 = j 

				//order_diff := math.Log(math.Abs(float64(ord1[i] - ord2[j]))+1.0)
				order_diff := math.Abs(float64(ord1[i] - ord2[j])) * 10.0

				if math.Abs(float64(ord1[i] - ord2[j])) == 0 {

					match.distance = distance(locs1[i], locs2_rotated[j]) + order_diff

					allMatch.ass = append(allMatch.ass, match)

				}
			}
		}


		sort.Sort(&allMatch)

		map1 := make(map[int]bool)
		map2 := make(map[int]bool)

		var total_distance float64 = 0.0 
		var number_of_match int = 0 

		match_c := 0
		for _, match := range allMatch.ass {
			// ignore the last matching (could be outliers)
			if match_c > len(locs1) -2 || match_c > len(locs2) - 2{
				break
			}

			l1 := match.l1 
			l2 := match.l2 

			if _,ok := map1[l1]; ok {
				continue 
			}

			if _,ok := map2[l2]; ok {
				continue 
			}

			map1[l1] = true 
			map2[l2] = true 

			if match.distance < 10.0 {
				number_of_match += 1 
			} else {
				//total_distance += match.distance
			}

			total_distance += match.distance
			match_c += 1
			// sanity check
			// if ind < 10 {
			// 	fmt.Println(ind, match.distance)
			// }

		}

		if total_distance - float64(number_of_match)*10.0 < best_min_distance {
			best_min_distance = total_distance - float64(number_of_match)*10.0
			best_angle = angle 
		}

		//fmt.Println("angle", angle, total_distance)
	//}

	//fmt.Println("best angle", best_angle / (2.0*3.1415926/64.0), a_l, a_h)



	trans.center = center2
	trans.angle = best_angle
	trans.scale = 1.0

	return trans, best_min_distance
}

func graphSimilarity(subgraph1, subgraph2 Subgraph) float64 {
	// TODO
	return 0.0
}

type graphMatch struct{
	graph  			*Subgraph
	trans  			Transformation
	similarityScore float64
}

type graphMatchList struct {
	list  []graphMatch
}

func (a *graphMatchList) Len() int {
	return len(a.list)
}

func (a *graphMatchList) Swap(i,j int) {

	a.list[i], a.list[j] = a.list[j], a.list[i]

}

func (a *graphMatchList) Less(i,j int) bool {
	return a.list[i].similarityScore > a.list[j].similarityScore 
}


func ComputeMatchingScore(graph1 Subgraph, graph2 Subgraph) (Transformation, float64) {
	defer printtime(makeTimestamp(), "ComputeMatchingScore")
	var score float64 = 0

	// rotation translation invarient similarity
	trans, score_rti := fastTransformationMatch(graph1, graph2)
	score_rti = math.Pow(2.0, -score_rti/100.0)


	// distance similarity 
	var score_loc float64 = 0.0
	if trans.NoLocation == false {
		score_loc = 1.0/(distance(trans.bias, [3]float64{0.0, 0.0, 0.0})+1.0)
	}

	score = score_rti + score_loc

	return trans, score
}



func (dag *DAG) match(frontiernode *DAGNode,  targetDags []*DAG) *Prediction {
	defer printtime(makeTimestamp(), "match")

	//starttime := makeTimestamp()

	//printtime(starttime, "match_mark1")

	var minimal_match int = 10
	//var minimal_depth int = 4  // This was 4 (20190920). Changed to 0 in order to test cancellation 
	var minimal_depth int = MinimalDepth  // This was 4 (20190920). Changed to 0 in order to test cancellation 
	
	var maximal_depth int = 6
	//var maximal_match int = 10

	var candidateGraphs []Subgraph
	var candidateRemoved []bool

	var sourceGraph Subgraph

	sourceGraph.dag = dag 
	sourceGraph.startNode = frontiernode
	sourceGraph.terminateNode = frontiernode

	// find candidates (match one node)

	candidateGraphs = append(candidateGraphs, sourceGraph)
	candidateRemoved = append(candidateRemoved, false)

	candidateMask := make(map[string]bool)

	PrintLog(LogDebug, fmt.Sprintln("Target Dags Num", len(targetDags)))
	PrintLog(LogDebug, fmt.Sprintln("node", frontiernode.Name))

	for _,graph := range targetDags {
		for _,node := range graph.Name2Node {
			//fmt.Println(node.Name, frontiernode.Name,  len(node.Children))
			//fmt.Println(node.PureName, frontiernode.PureName, IsSameNode(frontiernode, node))

			if node == frontiernode {
				continue
			}
			if len(node.Children) == 0 {
				continue
			}

			tag := fmt.Sprintf("%p_%d", graph, node.Ordering)

			// (20190920) Remove this ... 
			// used for debugging, remove future nodes in the graph
			//if graph == dag && node.Ordering >= frontiernode.Ordering {
			//	continue
			//}



			if _, ok := candidateMask[tag]; ok {
				//(20190920 remove this) continue // TODO we may need this 
			} else {
				candidateMask[tag] = true
			}

			//fmt.Println(tag)

			if dag.debuglog {
				fmt.Println(tag, node.Name)
			}



			if IsSameNode(frontiernode, node) && frontiernode != node {
				var subgraph Subgraph

				subgraph.dag = graph 
				subgraph.startNode = node 
				subgraph.terminateNode = node 

				//fmt.Println("add candidate!")

				candidateGraphs = append(candidateGraphs, subgraph)
				candidateRemoved = append(candidateRemoved, false)
			}
		}
	}

	PrintLog(LogDebug, fmt.Sprintln("number of candicates", len(candidateGraphs)-1))

	//printtime(starttime, "match_mark2")
	// find exact topological matchings (memorization speed-up)
	// recursive matching (capsules)


	var nCandidate int = len(candidateGraphs) - 1

	var cc int = 0
	for {
		cc += 1
		if cc > maximal_depth {
			break
		}
		// extend the candicate graph

		var newSubGraph []Subgraph

		var stop bool = false
		tmp_remove := []bool{}

		for i:=0; i<len(candidateGraphs);i++ {
			var g Subgraph
			tmp_remove = append(tmp_remove, candidateRemoved[i])
			if candidateRemoved[i] == false{
				g.dag = candidateGraphs[i].dag 
				g.startNode = candidateGraphs[i].startNode
				g.terminateNode = candidateGraphs[i].terminateNode
				//fmt.Println(i,"candidateGraphs[i] before", candidateGraphs[i])

				if len(g.terminateNode.Parents) > 1 {
					g.terminateNode = g.dag.findCommonAscendant(g.terminateNode)
				} else if len(g.terminateNode.Parents) == 1 {
					g.terminateNode = g.terminateNode.Parents[0]
				} else {
					//fmt.Println("mark0", nCandidate)
					if i == 0 {
						//fmt.Println("mark1")
						stop = true
						break
					}

					tmp_remove[i] = true
					if minimal_match >= nCandidate && cc > minimal_depth {
						stop = true
					} else {
						nCandidate -= 1
					}
					
				}
				//fmt.Println(i,"candidateGraphs[i] after", candidateGraphs[i])


			}

			newSubGraph = append(newSubGraph, g)
		}

		if stop {
			break
		}

		for i:=1; i<len(candidateGraphs);i++ {
			candidateRemoved[i] = tmp_remove[i]
		}


		var source_delta_graph Subgraph

		source_delta_graph.dag = candidateGraphs[0].dag 
		source_delta_graph.startNode = candidateGraphs[0].terminateNode
		source_delta_graph.terminateNode = newSubGraph[0].terminateNode

		tmp_remove = []bool{false}

		for i:=1; i<len(candidateGraphs);i++ {
			tmp_remove = append(tmp_remove, candidateRemoved[i])
			if candidateRemoved[i] == false {
				var delta_graph Subgraph

				delta_graph.dag = candidateGraphs[i].dag 
				delta_graph.startNode = candidateGraphs[i].terminateNode
				delta_graph.terminateNode = newSubGraph[i].terminateNode
				//fmt.Println(i,"source_delta_graph", source_delta_graph)

				//t0 := makeTimestamp()

				if _recursive_match(source_delta_graph, delta_graph) == false {
				//if _recursive_match(newSubGraph[0], newSubGraph[i]) == false {
				
					//fmt.Println("mark2")
					tmp_remove[i] = true
					if minimal_match >= nCandidate && cc > minimal_depth {
						stop = true
					} else {
						nCandidate -= 1
					}
				}

				//printtime(t0, "_recursive_match")
			}
		}

		if stop {
			break
		}

		// oldCandidate <- newCandidate 


		for i:=0; i<len(candidateGraphs);i++ {
			candidateRemoved[i] = tmp_remove[i]
			if candidateRemoved[i] == false{
				candidateGraphs[i] = newSubGraph[i]
			}
		}

	}
	//printtime(starttime, "match_mark3")
	//fmt.Println("loop n",cc)
	// ranking by the similarity (context/locations/sensor values/dates)

	var graphMatchs []graphMatch

	for i:=1; i<len(candidateGraphs);i++{
		if candidateRemoved[i] == false {
			
			var match graphMatch

			match.graph = &(candidateGraphs[i])

			

			// TODO TODO  add different similarity function here
			trans, score := ComputeMatchingScore(candidateGraphs[0],candidateGraphs[i])
			if dag.debuglog || vis_log == true {
				fmt.Println(i,candidateGraphs[0].Size(),candidateGraphs[0].startNode.Name,candidateGraphs[0].terminateNode.Name)
				fmt.Println(i,candidateGraphs[i].Size(),candidateGraphs[i].startNode.Name,candidateGraphs[i].terminateNode.Name)
				fmt.Println("score", score, "trans", trans)
			}


			match.trans = trans 
			// TODO
			match.similarityScore = score

			graphMatchs = append(graphMatchs, match)
		}
	}





	allGraphMatch := graphMatchList{graphMatchs}

	sort.Sort(&allGraphMatch)

	// get top 10 ? matchings 


	nMatch := len(allGraphMatch.list)

	if nMatch == 0 {
		pred := new(Prediction)
		return pred
	}


	var maxscore float64 = allGraphMatch.list[0].similarityScore

	
	
	for i:=0; i<len(allGraphMatch.list); i++ {
		if allGraphMatch.list[i].similarityScore < 0.2 * maxscore {
			nMatch = i + 1
			break
		}
	}
	if nMatch > 10 {
		nMatch = 10
	}

	PrintLog(LogDebug, fmt.Sprintln(nMatch, "best matching score", maxscore))


	allGraphMatch.list = allGraphMatch.list[0:nMatch]

	//printtime(starttime, "match_mark4")
	var preds []*Prediction

	for i:=0; i<nMatch; i++ {
		prob := allGraphMatch.list[i].similarityScore/maxscore 

		_pred := NewPredictionFromSubGraph(*allGraphMatch.list[i].graph, allGraphMatch.list[i].trans, prob, 0.9, CFG_MAXNODE)
		if quiet_log == false {
			fmt.Println("match ",i,allGraphMatch.list[i].graph.startNode.Name, i,allGraphMatch.list[i].graph.terminateNode.Name,"size", allGraphMatch.list[i].graph.Size(), len(_pred.Nodes), allGraphMatch.list[i].trans, prob, allGraphMatch.list[i].similarityScore, maxscore)
		}
		//printtime(starttime, "match_mark5")
		if dag.debuglog || vis_log == true {
			DrawMatchAndPrediction(&candidateGraphs[0], allGraphMatch.list[i].graph, _pred, fmt.Sprintf("debug/matches%d.png",i))
			DrawMatch(&candidateGraphs[0], &allGraphMatch.list[i], fmt.Sprintf("debug/matchonly%d.png",i))
		}
		// if len(_pred.Nodes) > 0 {
		// 	fmt.Println(_pred.Nodes[0])
		// }

		preds = append(preds, _pred)
	}
	//printtime(starttime, "match_mark6")

	pred := MergePredictions(preds, CFG_MERGEP_REDICTION_RANGE, func(a,b float64) float64 {return a+b})

	pred.Normalization()

	// if len(pred.Nodes) > 0 {
	// 	fmt.Println("vis", pred.Nodes[0])
	// }

	if dag.debuglog {
		pred.DebugVisualization()
	}

	return pred 
}

func (dag *DAG) SampleDag() {


}


func (dag *DAG) SaveDag(filename string) {
	fo, _ := os.Create(filename)
	defer fo.Close()

	encoder := gob.NewEncoder(fo)
	encoder.Encode(dag.toDAGString())
}


func LoadDag(filename string) []*DAG {
	dag := new(DAGString)

	fo, err := os.Open(filename)
	defer fo.Close()

	if err != nil {
		return []*DAG{}
	}

	decoder := gob.NewDecoder(fo)
	decoder.Decode(dag)

	return []*DAG{dag.toDAG()} 
}




var DebugVisualizationLastDraw int64 = 0 

func (pred *Prediction) DebugVisualization() {

	t := makeTimestamp()

	if t < DebugVisualizationLastDraw + 1000 {
		return 
	}

	DebugVisualizationLastDraw = t

	name := "os/web/prediction.png"
	img := image.NewRGBA(image.Rectangle{image.Point{0,0}, image.Point{800,800}})
	gc := draw2dimg.NewGraphicContext(img)
	gc.SetStrokeColor(color.RGBA{0xff, 0xff, 0xff, 0xff})
	gc.SetLineWidth(3)

	var scale float64 = 4.0 

	for _, node := range pred.Nodes {
		loc := node.Location

		x := loc[0] * scale + 400.0 
		y := loc[1] * scale + 400.0


		c := 255 - uint8(node.Prob * 255.0)

		gc.SetStrokeColor(color.RGBA{0xff, c, c, 0xff})


		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()



		gc.SetStrokeColor(color.RGBA{0x00, 0x00, 0xff, 0xaa})
		gc.SetFillColor(color.RGBA{0x00, 0x00, 0xff, 0x11})

		if node.CancelOtherTask == true {
			//fmt.Println("draw circle?", x,y,node.CancelOtherTaskRadius * scale)
			draw2dkit.Circle(gc, x, y, node.CancelOtherTaskRadius * scale)
		}
	}



	f, err := os.Create(name)
	if err != nil {
		fmt.Println("DebugVisualization Error", err)
	} else {
		png.Encode(f, img)	
		f.Close()
	}


}


func DrawLocations(locs [][3]float64, name string) {
	img := image.NewRGBA(image.Rectangle{image.Point{0,0}, image.Point{800,800}})
	gc := draw2dimg.NewGraphicContext(img)
	gc.SetStrokeColor(color.RGBA{0xff, 0x00, 0x00, 0xff})
	gc.SetLineWidth(3)


	for _, loc := range locs {
		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()
	}



	f, err := os.Create(name)
	if err != nil {
		fmt.Println("DrawLocation Error", err)
	} else {
		png.Encode(f, img)	
		f.Close()
	}


}



func DrawMatchAndPrediction(frontier *Subgraph, matched_graph *Subgraph, pred *Prediction, name string) {
	

	img := image.NewRGBA(image.Rectangle{image.Point{0,0}, image.Point{800,800}})
	gc := draw2dimg.NewGraphicContext(img)


	locs := frontier.Locations()

	gc.SetStrokeColor(color.RGBA{0xff, 0x00, 0x00, 0xff})
	gc.SetLineWidth(3)

	for _, loc := range locs {
		

		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()
	}


	locs = matched_graph.Locations()

	gc.SetStrokeColor(color.RGBA{0x00, 0x00, 0xff, 0xff})
	gc.SetLineWidth(3)

	for _, loc := range locs {
		

		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()
	}




	gc.SetStrokeColor(color.RGBA{0x00, 0xff, 0x00, 0xff})
	gc.SetLineWidth(3)

	for _, node := range pred.Nodes {
		loc := node.Location
		
		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()
	}



	_pred := NewPredictionFromSubGraph(*matched_graph, Transformation{}, 1.0, 0.9, CFG_MAXNODE )

	gc.SetStrokeColor(color.RGBA{0xff, 0xff, 0x00, 0xff})
	gc.SetLineWidth(3)

	for _, node := range _pred.Nodes {
		loc := node.Location
		
		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()
	}




	f, err := os.Create(name)
	if err != nil {
		fmt.Println("DrawMatchAndPrediction Error", err)
	} else {
		png.Encode(f, img)	
		f.Close()
	}

}

func DrawMatch(frontier *Subgraph, match *graphMatch, name string) {
	
	img := image.NewRGBA(image.Rectangle{image.Point{0,0}, image.Point{800,800}})
	gc := draw2dimg.NewGraphicContext(img)


	locs := frontier.Locations()

	gc.SetStrokeColor(color.RGBA{0xff, 0x00, 0x00, 0xff})
	gc.SetLineWidth(3)

	for _, loc := range locs {
		

		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()
	}


	locs = match.graph.Locations()

	gc.SetStrokeColor(color.RGBA{0x00, 0x00, 0xff, 0x88})
	gc.SetLineWidth(3)

	for _, _loc := range locs {
		
		loc := ApplyTransformation(_loc, match.trans)

		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()
	}

	locs = match.graph.Locations()

	gc.SetStrokeColor(color.RGBA{0x00, 0xff, 0x00, 0x88})
	gc.SetLineWidth(3)

	for _, _loc := range locs {
		
		loc := _loc


		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()
	}


	f, err := os.Create(name)
	if err != nil {
		fmt.Println("DrawMatch Error", err)
	} else {
		png.Encode(f, img)	
		f.Close()
	}


}

