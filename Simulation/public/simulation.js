// Canvas element and context to manipulate
var canvas = document.getElementById("simCanvas");
ctx = canvas.getContext("2d");

// Car and robot array that will be populated later on
var cars = [];
var robots = [];

var ROUTE = "";

var TESTSTEP = 0;

var PREVIEW_SIZE = 25;

// Height and width of one grid in pixels.
var GRID_WIDTH = 25;
var GRID_HEIGHT = 50;

// Width and height of canvas, will be populated later.
var CANVAS_WIDTH = 0;
var CANVAS_HEIGHT = 0;

// How many grids are there in one row and one column
var GRID_W = 0;
var GRID_H = 0;

// The layout array
var LAYOUT = [];

// Variables needed to hold the steps
var STEPS = 0;
var STEPINDEX = 0;
var STEPCOUNT = 100;
var SPEED = 1;
var MOVING = false;

// Images of robots, cars, parking lot.
var robot_img = new Image();
robot_img.src = './public/assets/robots/robot_200.png';

var robot_shadow_img = new Image();
robot_shadow_img.src = './public/assets/robots/robot_200_shadow.png';


var noline_img = new Image();

var car_imgs = [];

var routeInstructions = [];

noline_img.src = './public/assets/parking_lot/noline_200.jpg';

// Basically a function class that will hold robot and car objects, make them move, stop etc.
function sprite(options) {
    var that = {},
        speedCount = 0;
		

    that.context = options.context;
    that.width = options.width;
    that.height = options.height;
	that.shadowW = options.width;
	that.shadowH = options.height;

    that.image = options.image;
    that.shadow = options.shadow;
    that.x = options.x;
    that.y = options.y;
	that.shadowX = options.shadowX;
	that.shadowY = options.shadowY;
    that.up = false;
	
	that.lifting = false;
    that.moving = false;
    that.dirX = options.dirX;
    that.dirY = options.dirY;
	that.direction = options.direction;
    /*
    Speed:
    1 - It takes 1 step to travel 1 tile e.g. Robot only EW movement
    2 - It takes 2 steps to travel 2 tiles e.g. Robot only NS movement + robot with car EW movement
    4 - It takes 4 steps to travel 4 tiles e.g. Robot with car NS movement
    */
    that.speed = options.speed;

    that.id = options.id;

    that.render = function() {
        if (that.moving) {
            that.update(that.dirX, that.dirY);
        }

        if (that.lifting) {
			that.context.drawImage(
                that.shadow,
                x = that.shadowX,
                y = that.shadowY,
				width = that.shadowW,
				height = that.shadowH
            )
            
        }
		
		that.context.drawImage(
			that.image,
			x = that.x,
			y = that.y,
			width = that.width,
			height = that.height
		)
    };
	
	// Update the sprite location if needed, add a step.
    that.update = function(dirX, dirY) {        
        speedCount += 1;
		
		if (that.direction == "E" || that.direction == "W"){
			if (that.speed == 1 || (that.speed == 2 && speedCount == 2) || (that.speed == 4 && speedCount == 4)) {
				that.addPixel(dirX, dirY);
			}
		} else if (that.direction == "N" || that.direction == "S"){
			if ((that.speed == 2 && speedCount == 1) || (that.speed == 4 && speedCount == 2)){
				that.addPixel(dirX, dirY);
			}
		} else if (that.direction == "L" && speedCount == 50){
			that.addPixel(dirX, dirY, false);
		} else if (that.direction == "D" && speedCount == 10){
			that.addPixel(dirX, dirY, false);
		}
		
		if (that.direction == "L"){
			that.width = that.width + (GRID_WIDTH * 0.0001);
			that.height = that.height + (GRID_HEIGHT * 0.0001);
		} else if (that.direction == "D"){
			that.width = that.width - (GRID_WIDTH * 0.0005);
			that.height = that.height - (GRID_HEIGHT * 0.0005);
		}
		

    }
	
	// Will add one pixel in desired direction. - on next update car is moving one pixel.
    that.addPixel = function(dirX, dirY, updateShadow = true) {

        if (dirX == 1) {
            that.x += 1 / SPEED;
			if (updateShadow == true){
				that.shadowX += 1 / SPEED;
			}
        }
		if (dirX == -1) {
            that.x -= 1 / SPEED;
			if (updateShadow == true){
				that.shadowX -= 1 / SPEED;
			}
        }
		if (dirY == 1) {
            that.y -= 1 / SPEED;
			if (updateShadow == true){
				that.shadowY -= 1 / SPEED;
			}
        }
		if (dirY == -1) {
            that.y += 1 / SPEED;
			if (updateShadow == true){
				that.shadowY += 1 / SPEED;
			}
        }
        speedCount = 0;
    }

    return that;
}

// Returns array of car images
function getCarImgs() {
    var car_imgs = [];

    for (var i = 1; i <= 3; i++) {
        var car_img = [];
        car_img.push(new Image());
        car_img[0].src = './public/assets/cars/car' + i + '_200.png';
        car_img.push(new Image());
        car_img[1].src = './public/assets/cars/car' + i + '_200_shadow.png';
        car_imgs.push(car_img);
    }

    return car_imgs;
}

// Adds a machine with a type and ID to the x and y coordinates.
function addMachine(type, xGrid, yGrid, id) {

    if (type == "C0") {
        var img = car_imgs[0][0];
        var shadowImg = car_imgs[0][1];
    } else if (type == "C1") {
        var img = car_imgs[1][0];
        var shadowImg = car_imgs[1][1];
    } else if (type == "C2") {
        var img = car_imgs[2][0];
        var shadowImg = car_imgs[2][1];
    } else if (type == "R") {
        var img = robot_img;
        var shadowImg = robot_shadow_img;
    } else {
        console.log("Problem with types in addMachine()");
    }

    var x = yGrid * GRID_WIDTH;
    var y = xGrid * GRID_HEIGHT;

    //console.log(xGrid, yGrid, x,y, id);

    if (type == "C0" || type == "C1" || type == "C2") {
        var car = sprite({
            context: canvas.getContext('2d'),
            width: GRID_WIDTH,
            height: GRID_HEIGHT,
            image: img,
            shadow: shadowImg,
            x: x,
            y: y,
			shadowX: x,
			shadowY: y,
            id: id
        });

        cars.push(car);
    } else if (type == "R") {
        var robot = sprite({
            context: canvas.getContext('2d'),
            width: GRID_WIDTH,
            height: GRID_HEIGHT,
            image: img,
            shadow: shadowImg,
            x: x,
            y: y,
			shadowX: x,
			shadowY: y,
            id: id
        });

        robots.push(robot);
    }
}

// Sprite for parking lot.
var noline = sprite({
    context: canvas.getContext('2d'),
    width: GRID_WIDTH,
    height: GRID_HEIGHT,
    image: noline_img
});

// stops all the movement in the canvas
function stopAllMovement() {
    for (var i = 0; i < cars.length; i++) {
        cars[i].moving = false;
    }
    for (var i = 0; i < robots.length; i++) {
        robots[i].moving = false;
    }
}

// Loop of the simulation - steps that will be taken every frame.
function simulationLoop() {
    window.requestAnimationFrame(simulationLoop);
	
    if (MOVING) {
        STEPS++;
    }
    ctx.clearRect(0, 0, canvas.width, canvas.height);
	
	var step = Math.ceil((STEPS) / STEPCOUNT);
	var momevementStep = Math.ceil((STEPS + 1) / STEPCOUNT);
	if (momevementStep > STEPINDEX){
		stopAllMovement();
	}

	if (step > STEPINDEX){
		console.log("Doing step number " + step + ".");
		stopAllMovement();
		moves(routeInstructions, step);
		STEPINDEX++;
	}
		
    createParkingLayout();
    renderMachines(robots);
    renderMachines(cars);
	
	TESTSTEP++;
	
	if (TESTSTEP == 100){
		var endingImage = canvasToImage(canvas);
		endingImage.id = "end";
		endingImage.width = PREVIEW_SIZE * GRID_W;
		endingImage.height = PREVIEW_SIZE * 2 * GRID_H;
		document.getElementById("endingPoint").appendChild(endingImage);
		populateParkingLot(canvas,ROUTE);
	} else if (TESTSTEP == 110){
		var startingImage = canvasToImage(canvas);
		startingImage.id = "start";
		startingImage.width = PREVIEW_SIZE * GRID_W;
		startingImage.height = PREVIEW_SIZE * 2 * GRID_H;
		document.getElementById("startingPoint").appendChild(startingImage);
		canvas.style.visibility = "visible";
	}
}

// Helper function to render all the machines in the machine array
function renderMachines(machineArray) {
    for (var i = 0; i < machineArray.length; i++) {
        machineArray[i].render();
    }
}

// Event listener to start the simulation when the user clicks on a car.
canvas.addEventListener("mousedown", selectCar, false);

// Helper function that returns position of mouse relative to canvas.
function getMousePos(canvas, evt) {
    var rect = canvas.getBoundingClientRect();
    return {
        x: evt.clientX - rect.left,
        y: evt.clientY - rect.top
    };
}

// Will select the car and starts movement.
function selectCar(e) {
    var pos = getMousePos(canvas, e);
    var posx = pos.x;
    var posy = pos.y;

    // To get the car in this position
    if (!MOVING) {
        for (var i = 0; i < cars.length; i++) {
            if (posx >= cars[i].x && posx < (cars[i].x + GRID_WIDTH) && posy >= cars[i].y && posy < (cars[i].y + GRID_HEIGHT)) {
                if (cars[i].up) {
                    cars[i].up = false;
                } else {
                    cars[i].up = true;
                }
                moves(routeInstructions, 0);
            }
        }
    } else {
        var step = Math.ceil((STEPS - 1) / 100);
        //console.log("Next step will be number " + (step + 1));
    }
}

// Will move the machine depending on the instructions and speed.
function moveMachine(machine, instruction, speed) {
    machine.moving = true;
    MOVING = true;
	machine.lifting = true;
    switch (instruction) {
        case "N":
            machine.dirX = 0;
            machine.dirY = 1;
			machine.direction = "N";
            machine.speed = speed;
            machine.update(0, 1);
            break;
        case "S":
            machine.dirX = 0;
            machine.dirY = -1;
			machine.direction = "S";
            machine.speed = speed;
            machine.update(0, -1);
            break;
        case "E":
            machine.dirX = 1;
            machine.dirY = 0;
			machine.direction = "E";
            machine.speed = speed;
            machine.update(1, 0);
            break;
        case "W":
            machine.dirX = -1;
            machine.dirY = 0;
			machine.direction = "W";
            machine.speed = speed;
            machine.update(-1, 0);
            break;
        case "L":
			machine.dirX = 1;
			machine.dirY = 1;
			machine.direction = "L";
			machine.update(1,1);
			machine.lifting = true;
			break;            
		case "D":
			machine.dirX = -1;
			machine.dirY = -1;
			machine.direction = "D";
			machine.update(-1,-1);
			machine.lifting = true;
			break;
    }
}

// Function that will move all the machines one instruction step at a time.
function moves(instructions, stepNr) {
    if (stepNr < instructions.length) {
        var stepArray = instructions[stepNr];
        for (var i = 0; i < stepArray.length; i++) {
            moveMachine(getMachine(stepArray[i][0]), stepArray[i][1], stepArray[i][2]);
			console.log(stepArray[i][0], stepArray[i][1], stepArray[i][2]);
        }
		if (stepArray.length == 0){
			console.log("No movement")
			MOVING = true;
		}
    } else {
        stopAllMovement();
    }
}

// Returns the machine object with the ID specified.
function getMachine(id) {
    if (id.charAt(0) == "R") {
        for (var i = 0; i < robots.length; i++) {
            if (robots[i].id == id) {
                return robots[i];
            }
        }
        console.log("Robot with id " + id + "not found!!");
    } else {
        for (var i = 0; i < cars.length; i++) {
            if (cars[i].id == id) {
                return cars[i];
            }
        }
        console.log("Car with id " + id + "not found!!");
    }
}


// Populates the parking lot by asking the layout of the parking lot from server.
function populateParkingLot(canvas, route, first = true) {
	if (first == true){
		var URL = "/Route/" + route + "/Layout/1";
	} else{
		var URL = "/Route/" + route + "/Layout/0";
	}    
    $.getJSON(URL, function(data) {
        GRID_H = data.height;
        GRID_W = data.width;
		//HERE SHOULD BE THE CODE TO CHECK THE SIZE OF THE PARKING LOT AND VIEWPORT TO DETERMINE THE SCALE SIZE OF PARKING LOT.
		var viewportWidth = window.innerWidth;
		var viewportHeight = window.innerHeight;
		setParkingLotScale(GRID_W, GRID_H, viewportWidth, viewportHeight);
        LAYOUT = data.layout;
        cars = [];
        robots = [];
		CANVAS_WIDTH = GRID_WIDTH * GRID_W;
        canvas.width = CANVAS_WIDTH;
		CANVAS_HEIGHT = GRID_HEIGHT * GRID_H; 
        canvas.height = CANVAS_HEIGHT;       
		createParkingLayout();
        for (var i = 0; i < data.machines.length; i++) {
            addMachine(data.machines[i][0], data.machines[i][1], data.machines[i][2], data.machines[i][3])
        }
    })
}

function setParkingLotScale(gridX, gridY, viewPortW, viewPortH){
	var previewWidth = PREVIEW_SIZE * gridX;
	var previewHeight = PREVIEW_SIZE * gridY;
	
	var scaling  = [4, 2, 1];
	
	for (var i = 0; i < scaling.length; i++){
		var lotWidth = gridX * 100 / scaling[i];
		var lotHeight = gridY * 200 / scaling[i];
		var totalHeight = previewHeight + lotHeight;
		if (totalHeight <= viewPortH && lotWidth <= viewPortW){
			GRID_WIDTH = 100 / scaling[i];
			GRID_HEIGHT = 200 / scaling[i];
			SPEED = scaling[i];
		}
	}
	console.log(SPEED);
}

// Function for getting instructions from server
function getInstructions(route) {
    var URL = "/Route/" + route + "/Instructions";
    $.getJSON(URL, function(data) {
        //console.log(data);
        routeInstructions = data;
    });
}

// Creates the parking lot
function createParkingLayout() {
    for (var i = 0; i < GRID_H; i++) {
        for (var j = 0; j < GRID_W; j++) {
            if (LAYOUT[i][j] == "P") {
                ctx.drawImage(noline_img, j * GRID_WIDTH, i * GRID_HEIGHT);
            }
        }
    }
};

function canvasToImage(canvas){
	var image = new Image();
	image.src = canvas.toDataURL("image/png");
	return image;
}

function addStartingEndingStates(route){
	ROUTE = route;
	populateParkingLot(canvas, route, false);	
	
	simulationLoop();
}

canvas.style.visibility = "hidden";
car_imgs = getCarImgs();

