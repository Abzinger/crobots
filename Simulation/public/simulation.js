var canvas = document.getElementById("simCanvas");
ctx = canvas.getContext("2d");

var cars = [];
var robots = [];

var GRID_WIDTH  = 100;
var GRID_HEIGHT = 200;

var CANVAS_WIDTH = 0;
var CANVAS_HEIGHT = 0;

var GRID_W = 0;
var GRID_H = 0;

var LAYOUT = [];

var STEPS = 1;
var MOVING = false;

var robot_img = new Image();
robot_img.src = './public/assets/robots/robot_200.png';

var robot_up_img = new Image();
robot_up_img.src = './public/assets/robots/robot_up_200.png';

var corner_img = new Image();
var line_center_img = new Image();
var noline_img = new Image();

var car_imgs = [];

corner_img.src = './public/assets/parking_lot/corner_200.jpg';
line_center_img.src = './public/assets/parking_lot/line_center_200.jpg';
noline_img.src = './public/assets/parking_lot/noline_200.jpg';

function sprite (options) {
	var that = {},
		speedCount = 0;
	
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
	/*
	Speed:
	1 - It takes 1 step to travel 1 tile e.g. Robot only EW movement
	2 - It takes 2 steps to travel 2 tiles e.g. Robot only NS movement + robot with car EW movement
	4 - It takes 4 steps to travel 4 tiles e.g. Robot with car NS movement
	*/
	that.speed = options.speed;
	
	
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
		speedCount += 1;
		console.log(that.speed);
		if (that.speed == 1 || (that.speed == 2 && speedCount == 2) || (that.speed == 4 && speedCount == 2)){
			console.log("Whatup");
			that.addPixel(dirX,dirY);
		}
		
						
	}
	
	that.addPixel = function (dirX, dirY){
		if (dirX == 1){
			that.x += 1;
		} else if (dirX == -1){
			that.x -= 1;
		} else if (dirY == 1){
			that.y -= 1;
		} else if (dirY == -1){
			that.y += 1;
		}
		speedCount = 0;
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

function getCarImgs(){
	var car_imgs = [];
	
	for (var i = 1; i <= 3; i++){
		var car_img = [];
		car_img.push(new Image());
		car_img[0].src = './public/assets/cars/car' + i + '_200.png';
		car_img.push(new Image());
		car_img[1].src = './public/assets/cars/car' + i + '_up_200.png';
		car_imgs.push(car_img);
	}
	
	return car_imgs;
}

function addMachine(type, xGrid, yGrid){
	
	if (type == "C0"){
		var img = car_imgs[0][0];
		var upImg = car_imgs[0][1];
	} else if (type == "C1"){
		var img = car_imgs[1][0];
		var upImg = car_imgs[1][1];
	} else if (type == "C2"){
		var img = car_imgs[2][0];
		var upImg = car_imgs[2][1];
	} else if (type == "R"){
		var img = robot_img;
		var upImg = robot_up_img;
	} else {
		console.log("Problem with types in addMachine()");
	}
	
	var x = yGrid * GRID_WIDTH;
	var y = xGrid * GRID_HEIGHT;
	
	console.log(xGrid, yGrid, x,y);
	
	if (type == "C0" || type == "C1" || type == "C2"){
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
	} else if (type == "R"){
		var robot = sprite({
			context: canvas.getContext('2d'),
			width: 100,
			height: 200,
			image: img,
			upimage: upImg,
			x: x,
			y: y
		});
		
		robots.push(robot);
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
	createParkingLayout();
	renderMachines(robots);
	renderMachines(cars);
}

function renderMachines(machineArray){
	for (var i = 0; i < machineArray.length; i++){
		machineArray[i].render();
	}
}

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
	} else {
		var step = Math.ceil((STEPS -1) / 100);
		console.log("Next step will be number " + (step + 1));
	}
}

function moveCar(direction, car, speed){
	car.moving = true;
	MOVING = true;
	switch(direction){
		case "N":
			car.endY = car.y - GRID_HEIGHT;
			car.dirX = 0;
			car.dirY = 1;
			car.speed = speed;
			car.update(0,1);
			break;
		case "S":
			car.endY = car.y + GRID_HEIGHT;
			car.dirX = 0;
			car.dirY = -1;
			car.speed = speed;
			car.update(0,-1);
			break;
		case "E":
			car.endX = car.x + GRID_WIDTH;
			car.dirX = 1;
			car.dirY = 0;
			car.speed = speed;
			car.update(1,0);
			break;
		case "W":
			car.endX = car.x - GRID_WIDTH;
			car.dirX = -1;
			car.dirY = 0;
			car.speed = speed;
			car.update(-1,0);
			break;
	}
}

var moveArray=[[["E",0,2],["N",3,4]],[["E",0,2],["N",3,4]],[["W",0,2],["N",3,4]],[["W",0,2],["N",3,4]]];

function movements(moveArray){
	//console.log(STEPS);
	var step = Math.ceil((STEPS -1) / 100);
	if (step < moveArray.length){
		var stepArray = moveArray[step];
		for (var j = 0; j < stepArray.length; j++){
			moveCar(stepArray[j][0],cars[stepArray[j][1]],stepArray[j][2]);
		}
	} else{
		stopAllMovement();
	}
}


function populateParkingLot(route){
	var URL = "/Route/" + route + "/Layout";
	$.getJSON( URL, function ( data ) {
		console.log(data);
		GRID_H = data.height;
		GRID_W = data.width;
		LAYOUT = data.layout;
		cars = [];
		robots = [];
		console.log(LAYOUT);
		canvas.width = GRID_WIDTH * GRID_W;
		canvas.height = GRID_HEIGHT * GRID_H;
		createParkingLayout();
		car_imgs = getCarImgs();
		for (var i = 0; i < data.machines.length; i++){
			addMachine(data.machines[i][0], data.machines[i][1], data.machines[i][2])
		}
	})
}

function createParkingLayout(){
	for (var i = 0; i < GRID_H; i++){
		for (var j = 0; j < GRID_W; j++){
			if (LAYOUT[i][j] == "P"){	
				ctx.drawImage(noline_img, j*GRID_WIDTH, i*GRID_HEIGHT);
			}			
		}
	}
};


noline_img.addEventListener("load", gameLoop);

