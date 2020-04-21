var pi = Math.PI;
var tau = 2 * pi;

var width = 800
var height = 800


var svg = d3.select("#main").select("svg")
	.attr("width", width)
	.attr("height", height);


// var svg2 = d3.select("#spec").append("svg")
// 	.attr("width", 400)
// 	.attr("height", 400);



svg.append("svg:defs").append("svg:marker")
    .attr("id", "triangle")
    .attr("refX", 6)
    .attr("refY", 6)
    .attr("markerWidth", 30)
    .attr("markerHeight", 30)
    .attr("markerUnits","userSpaceOnUse")
    .attr("orient", "auto")
    .append("path")
    .attr("d", "M 0 0 12 6 0 12 3 6")
    .style("fill", "#FF00FF");


var bkLayer = svg.append("g")
	.attr('class', 'bk');

// var bkLayer2 = svg2.append("g")
// 	.attr('class', 'bk2');

function getUrlVars() {
    var vars = {};
    var parts = window.location.href.replace(/[?&]+([^=&]+)=([^&]*)/gi, function(m,key,value) {
        vars[key] = value;
    });
    return vars;
}

function updateimage() {

	$.get('/updatebk', function(data){

		console.log("get", data);

		var img = bkLayer.selectAll("image").data([Math.random()]);
		
		env = getUrlVars()["env"]

		img.enter().append("image")
		.attr('x', 0)
		.attr('y', 0)
		.attr('width', width)
		.attr('height', height)
		.merge(img)
		.attr("xlink:href", function(d) {console.log("bk.png"); return "bkgrid.jpg"; } )
		//.attr("xlink:href", function(d) {console.log("bk_"+env+".png?"+d); return "bk_"+env+".png?"+d; } )


		img.exit().remove();

	// 	var img2 = svg2.selectAll("image").data([Math.random()]);
	// 	console.log(img2);

	// 	img2.enter().append("image")
	// .attr('x', 0)
	// .attr('y', 0)
	// .attr('width', 400)
	// .attr('height', 400)
	// .merge(img2)
	// .attr("xlink:href", function(d) {console.log("prediction.png?"+d); return "prediction.png?"+d; } )


	// 	img2.exit().remove();

	});
}

setTimeout(updateimage, 5);
setInterval(updateimage, 5000);



var raster = svg.append("g");
var eventlayer = svg.append('rect')
	.attr('x', 0)
	.attr('y', 0)
	.attr('width', width)
	.attr('height', height)
	.attr('fill-opacity', 0)
	.attr('class', 'eventlayer');

var homesLayer = svg.append("g")
	.attr('class', 'homes');


var assignmentsLayer = svg.append("g")
	.attr('class', 'assignments');

var tasksLayer = svg.append("g")
	.attr('class', 'tasks');

var tasksLayerPolyline = svg.append("g")
	.attr('class', 'taskspolyline');

var dronesLayer = svg.append("g")
	.attr('class', 'drones');

var dronePathLayer = svg.append("g")
	.attr('class', 'dronepath');



var refreshDrones = Drones(dronesLayer, refresh);

var refreshFuncs = {
	'drones': refreshDrones,
};



function refresh(targets) {
	var targetSet = {};
	if(targets) {
		targets.forEach(function(target) {
			targetSet[target] = true;
		});
	}

	for(var k in refreshFuncs) {
		if(!targets || k in targetSet) {
			refreshFuncs[k]();
		}
	}
}





function Drones(vector, refresh) {
	var size = 6;
	var drones = [];

	var drone_path1 = [];
	var drone_path2 = [];

	var min_ddd = 10000.0;




	var getDrones = function() {
		$.get('/drones', function(data) {
			drones = data;




			if (drones.length > 0) {
				drone_path1.push([data[0].X, data[0].Y]);
			}

			if (drones.length > 1) {
				drone_path2.push([data[1].X, data[1].Y]);

				var a = data[0].X-data[1].X;
				var b = data[0].Y-data[1].Y;
				var c = data[0].Z-data[1].Z;

				var ddd = Math.sqrt(a*a + b*b + c*c);

				if (ddd < min_ddd) {
					min_ddd = ddd;
				}
				console.log("distance between the first two drones", ddd, "min", min_ddd);

			}

			if (drones.length > 2) {
				//drone_path1 = []
				//drone_path2 = []
			} else {
				if (drone_path1.length > 1024) {
					drone_path1.shift()
				}

				if (drone_path2.length > 1024) {
					drone_path2.shift()
				}
			}
			// console.log(data)
			refresh(['drones']);
		}, 'json');

	};

	setTimeout(getDrones, 1);
	setInterval(getDrones, 500);

	return function() {
		var color;
	
		color = 'blue';
		color_list = ['blue','green','lightseagreen','red','orange','teal','olive','aqua','fuchsia'];
		
		var vectorDrones = vector.selectAll('rect')
			.data(drones);

		console.log(drones)

		vectorDrones.enter()
			.append('rect')
			.attr('width', function(d) {return size; })
			.attr('height', function(d) {return size;})
			.merge(vectorDrones)
			.attr('x', function(d) { return 4*d.Y+width/2-size/2; })
			.attr('y', function(d) { return -4*d.X+height/2-size/2; })
			.attr('fill', function(d) {return color_list[d.ID]})
			.attr('stroke', function(d) {if (d.Phase == 2){ return 'red';} else {return 'white';} })
			.attr('stroke-width', function(d) {if (d.Phase == 2){ return 3;} else {return 1;} })
		vectorDrones.exit()
			.remove();



		var dronePath = vector.selectAll('path')
			.data([[drone_path1,0], [drone_path2,1]]);

		var linefunction = d3.line().x(function(d) {return 4*d[1] + width/2;}).y(function(d) {return -4*d[0] + width/2;}).curve(d3.curveBasis);

		//console.log(drone_path)

		dronePath.enter()
			.append('path')
			.merge(dronePath)
			//.attr("marker-end", "url(#triangle)")
			.attr('d', function(d) { return linefunction(d[0]);})
			.attr("stroke-width", 1)
			.attr("fill", "none")
			.attr("stroke", function(d) {return color_list[d[1]]});


		dronePath.exit()
			.remove();





	};
}

