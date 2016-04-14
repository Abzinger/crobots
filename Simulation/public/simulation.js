// Canvas element and context to manipulate
var canvas = document.getElementById("simCanvas");
ctx = canvas.getContext("2d");

// Car and robot array that will be populated later on
var cars = [];
var robots = [];

// Height and width of one grid in pixels.
var GRID_WIDTH = 100;
var GRID_HEIGHT = 200;

// Width and height of canvas, will be populated later.
var CANVAS_WIDTH = 0;
var CANVAS_HEIGHT = 0;

// How many grids are there in one row and one column
var GRID_W = 0;
var GRID_H = 0;

// The layout array
var LAYOUT = [];

// Variables needed to hold the steps - ghost steps is for the case where in one instruction set there is no movement at all
var STEPS = 1;
var GHOSTSTEPS = 0;
var GHOSTMOVEMENT = false;
var MOVING = false;

// Images of robots, cars, parking lot.
var robot_img = new Image();
robot_img.src = './public/assets/robots/robot_200.png';

var robot_up_img = new Image();
robot_up_img.src = './public/assets/robots/robot_up_200.png';

var corner_img = new Image();
var line_center_img = new Image();
var noline_img = new Image();

var car_imgs = [];

var routeInstructions = [];

corner_img.src = './public/assets/parking_lot/corner_200.jpg';
line_center_img.src = './public/assets/parking_lot/line_center_200.jpg';
noline_img.src = './public/assets/parking_lot/noline_200.jpg';

// Basically a function class that will hold robot and car objects, make them move, stop etc.
function sprite(options) {
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

    that.id = options.id;

    that.render = function() {
        if (that.moving) {
            that.update(that.dirX, that.dirY);
        }

        if (that.up) {
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
	
	// Update the sprite location if needed, add a step.
    that.update = function(dirX, dirY) {
        if (that.x == that.endX && that.y == that.endY) {
            //console.log(STEPS);
            var step = Math.ceil((STEPS - 1) / 100);
            console.log("Update step: " + step);
            if (step < routeInstructions.length) {
                moves(routeInstructions);
            } else {
                that.moving = false;
                MOVING = false;
                stopAllMovement();
            }
        }
        speedCount += 1;

        //console.log(that.speed + ", " + speedCount);
        if (that.speed == 1 || (that.speed == 2 && speedCount == 2) || (that.speed == 4 && speedCount == 2)) {
            that.addPixel(dirX, dirY);
        }


    }
	
	// Will add one pixel in desired direction. - on next update car is moving one pixel.
    that.addPixel = function(dirX, dirY) {

        if (dirX == 1) {
            that.x += 1;
        } else if (dirX == -1) {
            that.x -= 1;
        } else if (dirY == 1) {
            that.y -= 1;
        } else if (dirY == -1) {
            that.y += 1;
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
        car_img[1].src = './public/assets/cars/car' + i + '_up_200.png';
        car_imgs.push(car_img);
    }

    return car_imgs;
}

// Adds a machine with a type and ID to the x and y coordinates.
function addMachine(type, xGrid, yGrid, id) {

    if (type == "C0") {
        var img = car_imgs[0][0];
        var upImg = car_imgs[0][1];
    } else if (type == "C1") {
        var img = car_imgs[1][0];
        var upImg = car_imgs[1][1];
    } else if (type == "C2") {
        var img = car_imgs[2][0];
        var upImg = car_imgs[2][1];
    } else if (type == "R") {
        var img = robot_img;
        var upImg = robot_up_img;
    } else {
        console.log("Problem with types in addMachine()");
    }

    var x = yGrid * GRID_WIDTH;
    var y = xGrid * GRID_HEIGHT;

    //console.log(xGrid, yGrid, x,y, id);

    if (type == "C0" || type == "C1" || type == "C2") {
        var car = sprite({
            context: canvas.getContext('2d'),
            width: 100,
            height: 200,
            image: img,
            upimage: upImg,
            x: x,
            y: y,
            id: id
        });

        cars.push(car);
    } else if (type == "R") {
        var robot = sprite({
            context: canvas.getContext('2d'),
            width: 100,
            height: 200,
            image: img,
            upimage: upImg,
            x: x,
            y: y,
            id: id
        });

        robots.push(robot);
    }
}

// Was used for testing, returns random integer, deprecated.
function randomIntFromInterval(min, max) {
    return Math.floor(Math.random() * (max - min + 1) + min);
}

// Sprite for parking lot.
var noline = sprite({
    context: canvas.getContext('2d'),
    width: 100,
    height: 200,
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
    if (GHOSTMOVEMENT) {
        MOVING = true;
        GHOSTSTEPS += 1;
        stopAllMovement();
    }
    if (GHOSTSTEPS == 100) {
        MOVING = false;
        GHOSTMOVEMENT = false;
        GHOSTSTEPS = 0;
        moves(routeInstructions);
    }
    if (MOVING) {
        STEPS += 1;
    }
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    createParkingLayout();
    renderMachines(robots);
    renderMachines(cars);
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
                moves(routeInstructions);
            }
        }
    } else {
        var step = Math.ceil((STEPS - 1) / 100);
        console.log("Next step will be number " + (step + 1));
    }
}

// Will move the machine depending on the instructions and speed.
function moveMachine(machine, instruction, speed) {
    machine.moving = true;
    MOVING = true;
    switch (instruction) {
        case "N":
            machine.endY = machine.y - GRID_HEIGHT;
            machine.dirX = 0;
            machine.dirY = 1;
            machine.speed = speed;
            machine.update(0, 1);
            break;
        case "S":
            machine.endY = machine.y + GRID_HEIGHT;
            machine.dirX = 0;
            machine.dirY = -1;
            machine.speed = speed;
            machine.update(0, -1);
            break;
        case "E":
            machine.endX = machine.x + GRID_WIDTH;
            machine.dirX = 1;
            machine.dirY = 0;
            machine.speed = speed;
            machine.update(1, 0);
            break;
        case "W":
            machine.endX = machine.x - GRID_WIDTH;
            machine.dirX = -1;
            machine.dirY = 0;
            machine.speed = speed;
            machine.update(-1, 0);
            break;
        case "L":
            break;
    }
}

// Function that will move all the machines one instruction step at a time.
function moves(instructions) {
    var step = Math.ceil((STEPS - 1) / 100);
    console.log("Step number " + step + ", length of instructions: " + instructions.length);
    if (step < instructions.length) {
        var stepArray = instructions[step];
        for (var i = 0; i < stepArray.length; i++) {
            moveMachine(getMachine(stepArray[i][0]), stepArray[i][1], stepArray[i][2]);
        }
        if (stepArray.length == 0) {
            console.log("BOO! " + step);
            GHOSTMOVEMENT = true;
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
function populateParkingLot(route) {
    var URL = "/Route/" + route + "/Layout";
    $.getJSON(URL, function(data) {
        GRID_H = data.height;
        GRID_W = data.width;
        LAYOUT = data.layout;
        cars = [];
        robots = [];
        //console.log(LAYOUT);
        canvas.width = GRID_WIDTH * GRID_W;
        canvas.height = GRID_HEIGHT * GRID_H;
        createParkingLayout();
        car_imgs = getCarImgs();
        for (var i = 0; i < data.machines.length; i++) {
            //console.log(data.machines[i])
            addMachine(data.machines[i][0], data.machines[i][1], data.machines[i][2], data.machines[i][3])
        }
    })
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


noline_img.addEventListener("load", simulationLoop);