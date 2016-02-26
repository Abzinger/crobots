var simulation = new Phaser.Game(1600, 800, Phaser.AUTO, 'simulation', { preload: preload, create: create, update: update });

var GRID_WIDTH  = 100;
var GRID_HEIGHT = 200;
var VELOCITY = 30;

var isOver;

function preload() {
	simulation.load.image('corner', './public/assets/parking_lot/corner_200.jpg');
	simulation.load.image('line_center', './public/assets/parking_lot/line_center_200.jpg');
	simulation.load.image('noline', './public/assets/parking_lot/noline_200.jpg');
	
	simulation.load.image('car1', './public/assets/cars/car1_200.png');
	simulation.load.image('car2', './public/assets/cars/car2_200.png');
	simulation.load.image('car3', './public/assets/cars/car3_200.png');
	simulation.load.image('car4', './public/assets/cars/car4_200.png');
	simulation.load.image('car5', './public/assets/cars/car5_200.png');
	simulation.load.image('car6', './public/assets/cars/car6_200.png');
	
	simulation.load.image('robot', './public/assets/robots/robot_200.png');
	
}

function create() {
		
	createParkingLot();
	
	r = simulation.add.sprite(150,300,'robot');
	r.anchor.setTo(0.5,0.5);
	r.inputEnabled = true;
	
	s = simulation.add.sprite(150,300,'car1');
	s.anchor.setTo(0.5,0.5);
	s.inputEnabled = true;
	
	
	t = simulation.add.sprite(250,300,'car2');
	t.anchor.setTo(0.5,0.5);
	t.inputEnabled = true;
	
	
	simulation.add.sprite(300,400,'car3');
	simulation.add.sprite(1000,600,'car4');
	simulation.add.sprite(500,400,'car5');
	simulation.add.sprite(300,0,'car6');
	
	
}

function createParkingLot() {
	// 8 * 4
	for (var i = 0; i < 16; i++){
		for (var j = 0; j < 4; j++){
			simulation.add.sprite(i*100, j*200, 'noline');
		}
	}
}

function sleep(milliseconds) {
  var start = new Date().getTime();
  for (var i = 0; i < 1e7; i++) {
    if ((new Date().getTime() - start) > milliseconds){
      break;
    }
  }
}

function moveTo(object, direction, spaces){
	var startX = object.x;
	var startY = object.y;
	
	if (direction == 'E'){		
			
	} else if (direction == 'W'){
		simulation.physics.arcade.moveToXY(object,startX - (GRID_WIDTH * spaces), startY, VELOCITY);
			
	} else if (direction == 'N'){
		simulation.physics.arcade.moveToXY(object,startX, startY + (GRID_HEIGHT * spaces), VELOCITY);
			
	} else if (direction == 'S'){
		simulation.physics.arcade.moveToXY(object,startX, startY -(GRID_HEIGHT * spaces), VELOCITY);
		
	}

	
};

function update() {	
	isOver = s.input.pointerOver()? true : false;
	
	if (isOver){
		simulation.input.onDown.add(moveAround,this);
	}
	
	s.body.velocity.x = 0;
	s.body.velocity.y = 0;
	s.body.angularVelocity = 0;
}

function getInstructions(e){
	if (e == 'A'){
		
	} else if (e == 'B'){
		
	}
}

function moveAround(){	
	moveTo(s, 'N', 1);

}
