var canvas = document.getElementById("simCanvas");
ctx = canvas.getContext("2d");

var cars = [];
var robots = [];

var GRID_WIDTH  = 100;
var GRID_HEIGHT = 200;

var CANVAS_WIDTH = 1200;
var CANVAS_HEIGHT = 600;

var STEPS = 1;
var MOVING = false;


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
		frameIndex = 0;
	
	that.context = options.context;
	that.width = options.width;
	that.height = options.height;
	
	that.image = options.image;
	that.upimage = options.upimage;
	that.x = options.x;
	that.y = options.y;
	that.up = false;
	
	that.moving = false;
	that.endX = options.x;
	that.endY = options.y;
	that.dirX = options.dirX;
	that.dirY = options.dirY;
	
	that.render = function () {
		if (that.moving){
			that.update(that.dirX, that.dirY);
		}
		
		if (that.up){
			that.context.drawImage(
			that.upimage,
			x = that.x,
			y = that.y
			)
		} else {
			that.context.drawImage(
			that.image,
			x = that.x,
			y = that.y
			)
		}
		
	};
	
	that.loop = options.loop;
	
	that.update = function (dirX, dirY) {
		if (that.x == that.endX && that.y == that.endY){
			console.log(STEPS);
			var step = (STEPS) / 100;			
			if (step < moveArray.length){
				movements(moveArray);
			} else {
				that.moving = false;
				MOVING = false;
				stopAllMovement();
			}
		}
		if (dirX == 1){
			that.x += 1;
		} else if (dirX == -1){
			that.x -= 1;
		} else if (dirY == 1){
			that.y -= 2;
		} else if (dirY == -1){
			that.y += 2;
		}				
	}
	return that;
}

function addCars(amount){
	var car_imgs = [];
	
	for (var i = 1; i <= amount; i++){
		var car_img = [];
		var carInt = randomIntFromInterval(1,6);
		car_img.push(new Image());
		car_img[0].src = './public/assets/cars/car' + carInt + '_200.png';
		car_img.push(new Image());
		car_img[1].src = './public/assets/cars/car' + carInt + '_up_200.png';
		car_imgs.push(car_img);
	}
	
	var coords = [];
	
	var placesX = CANVAS_WIDTH / GRID_WIDTH;
	var placesY = CANVAS_HEIGHT / GRID_HEIGHT;
	
	for (var i = 0; i < amount; i++){
		
		do {
			var existing = false;
			var x = randomIntFromInterval(0,placesX - 1) * GRID_WIDTH;
			var y = randomIntFromInterval(0, placesY - 1) * GRID_HEIGHT;
			
			for (var k = 0; k < coords.length; k++){
				if (coords[k][0] == x && coords[k][1] == y){					
					existing = true;
				}
			}
		}
		while (existing);
		coords.push([x,y]);		
	}	
	
	
	for (var i = 0; i < amount; i++){
		var img = car_imgs[i][0];
		var upImg = car_imgs[i][1];
		var x = coords[i][0];
		var y = coords[i][1]
		var car = sprite({
		context: canvas.getContext('2d'),
		width: 100,
		height: 200,
		image: img,
		upimage: upImg,
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

function stopAllMovement(){
	for (var i = 0; i < cars.length; i++){
		cars[i].moving = false;
	}
}
function gameLoop() {
	window.requestAnimationFrame(gameLoop);
	if (MOVING){
		STEPS += 1;
	}
	ctx.clearRect(0, 0, canvas.width, canvas.height);
	populateParkingLot();
	renderCars();
}

function renderCars(){
	for (var i = 0; i < cars.length; i++){
		cars[i].render();
	}
}

noline_img.onload = populateParkingLot();

function populateParkingLot(){
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

canvas.addEventListener("mousedown", selectCar, false);

function getMousePos(canvas, evt) {
    var rect = canvas.getBoundingClientRect();
    return {
      x: evt.clientX - rect.left,
      y: evt.clientY - rect.top
    };
}

function selectCar(e){
	var pos = getMousePos(canvas,e);
	var posx = pos.x;
	var posy = pos.y;
	
	// To get the car in this position
	if (!MOVING){	
		for (var i = 0; i < cars.length; i++){
			if (posx >= cars[i].x && posx < (cars[i].x + GRID_WIDTH) && posy >= cars[i].y && posy < (cars[i].y + GRID_HEIGHT)){
				if (cars[i].up){
					cars[i].up = false;
				} else {
					cars[i].up = true;
				}
				movements(moveArray);
			}
		}
	}
}

function moveCar(direction, car){
	car.moving = true;
	MOVING = true;
	switch(direction){
		case "N":
			car.endY = car.y - GRID_HEIGHT;
			car.dirX = 0;
			car.dirY = 1;
			car.update(0,1);
			break;
		case "S":
			car.endY = car.y + GRID_HEIGHT;
			car.dirX = 0;
			car.dirY = -1;
			car.update(0,-1);
			break;
		case "E":
			car.endX = car.x + GRID_WIDTH;
			car.dirX = 1;
			car.dirY = 0;
			car.update(1,0);
			break;
		case "W":
			car.endX = car.x - GRID_WIDTH;
			car.dirX = -1;
			car.dirY = 0;
			car.update(-1,0);
			break;
	}
}

var moveArray=[[["E",0],["N",1]],[["S",0],["E",1]],[["W",0],["S",1]],[["N",0],["W",1]]];

function movements(moveArray){
	//console.log(STEPS);
	var step = Math.ceil((STEPS -1) / 100);
	if (step < moveArray.length){
		var stepArray = moveArray[step];
		for (var j = 0; j < stepArray.length; j++){
			moveCar(stepArray[j][0],cars[stepArray[j][1]]);
		}
	} else{
		stopAllMovement();
	}
}

