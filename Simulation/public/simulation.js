var simulation = new Phaser.Game(1600, 800, Phaser.AUTP, 'simulation', { preload: preload, create: create, update: update });

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
	
}

function create() {
	
	createParkingLot();
	
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

function update() {
	
	simulation.physics.enable(s, Phaser.Physics.ARCADE);
	
	isOver = s.input.pointerOver()? true : false;

	if (simulation.input.activePointer.leftButton.isDown){
		if (isOver){
			moveAround();
		}
	}
}

function getInstructions(e){
	if (e == 'A'){
		
	} else if (e == 'B'){
		
	}
}

function moveAround(){	
	tween = simulation.add.tween(s).to({ x: [50,50,50,650,650], y:[300, 500, 700,700,500]}, 5000, 'Linear', false, 0);
	tween2 = simulation.add.tween(t).to({ x: [150], y:[300]}, 100, 'Linear', false, 0);
	
	tween.chain(tween2);
		
	tween.start();

}
