var canvas = document.getElementById("simCanvas");
ctx = canvas.getContext("2d");

var cars = [];
var robots = [];

var GRID_WIDTH  = 100;
var GRID_HEIGHT = 200;

var CANVAS_WIDTH = 1200;
var CANVAS_HEIGHT = 600;


var robot_img = new Image();
robot_img.src = './public/assets/robots/robot_200.png';

var corner_img = new Image();
var line_center_img = new Image();
var noline_img = new Image();

corner_img.src = './public/assets/parking_lot/corner_200.jpg';
line_center_img.src = './public/assets/parking_lot/line_center_200.jpg';
noline_img.src = './public/assets/parking_lot/noline_200.jpg';

function sprite (options) {
	var that = {},
		frameIndex = 0,
		tickCount = 0,
		ticksPerFrame = 0,
		numberOfFrames = options.numberOfFrames || 1;
	
	that.context = options.context;
	that.width = options.width;
	that.height = options.height;
	
	that.image = options.image;
	that.x = options.x;
	that.y = options.y;
	
	that.render = function () {
		that.context.drawImage(
			that.image,
			x = that.x,
			y = that.y
		)
	};
	
	that.loop = options.loop;
	
	that.update = function () {
		
		tickCount += 1;
		
		if (tickCount > ticksPerFrame) {
			
			tickCount = 0;
			
			if (frameIndex < numberOfFrames - 1){
				
				// Next frame
				frameIndex += 1;
			} else if (that.loop) {
				frameIndex = 0;
			}
			
		}
		
	}
	
	
	
	return that;
}

function addCars(amount){
	var car_imgs = []
	for (var i = 0; i < 6; i++){
		car_imgs.push(new Image());
		car_imgs[i].src = './public/assets/cars/car' + (i + 1) + '_200.png';
		
	}
	
	var coords = [];
	
	var placesX = CANVAS_WIDTH / GRID_WIDTH;
	var placesY = CANVAS_HEIGHT / GRID_HEIGHT;
	
	for (var i = 0; i < amount; i++){
		var existing = false;
		do {
			var x = randomIntFromInterval(0,placesX - 1) * GRID_WIDTH;
			var y = randomIntFromInterval(0, placesY - 1) * GRID_HEIGHT;
			
			for (var k = 0; k < coords.length; k++){
				if (coords[k] == x && coords[k] == y){
					console.log (x + ";" + y);
					existing = true;
				}
			}
		}
		while (existing);
		coords.push([x,y]);		
	}	
	
	for (var i = 0; i < amount; i++){
		var img = car_imgs[randomIntFromInterval(0, car_imgs.length)];
		var x = coords[i][0];
		var y = coords[i][1]
		var car = sprite({
		context: canvas.getContext('2d'),
		width: 100,
		height: 200,
		image: img,
		x: x,
		y: y
		});
		
		cars.push(car);
		
	}

	for (var i = 0; i < cars.length; i++){
		cars[i].render();
	}
}

function randomIntFromInterval(min,max)
{
    return Math.floor(Math.random()*(max-min+1)+min);
}

var noline = sprite({
	context: canvas.getContext('2d'),
	width: 100,
	height: 200,
	image: noline_img
});

function gameLoop() {
	window.requestAnimationFrame(gameLoop);
	
	renderCars();
}

function renderCars(){
	for (var i = 0; i < cars.length; i++){
		cars[i].render();
	}
}

noline_img.onload = function(){
	for (var i = 0; i < 12; i++){
		for (var j = 0; j < 3; j++){
			if (i % 2 == 1){
				ctx.drawImage(noline_img,i*100, j*200);
			} else {
				ctx.drawImage(noline_img,i*100, j*200);
			}
		}
	}
	
};

$(document).ready(function(){
	addCars(10);
})



noline_img.addEventListener("load", gameLoop);




