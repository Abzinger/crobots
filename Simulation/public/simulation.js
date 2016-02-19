var simulation = new Phaser.Game(1000, 1000, Phaser.AUTP, 'simulation', { preload: preload, create: create, update: update });

var isOver;

function preload() {
	simulation.load.image('parkingLot', './public/assets/parkingLot.jpg');
	simulation.load.image('car', './public/assets/car.jpg');
	simulation.load.image('car1', './public/assets/carRed.jpg');
	simulation.load.image('car2', './public/assets/carRed.jpg');
	simulation.load.image('car3', './public/assets/carRed.jpg');
	simulation.load.image('car4', './public/assets/carRed.jpg');
	simulation.load.image('car5', './public/assets/carRed.jpg');
	simulation.load.image('car6', './public/assets/carRed.jpg');
	simulation.load.image('car7', './public/assets/carRed.jpg');
}

function create() {
	
	simulation.add.sprite(0,0,'parkingLot');
	
	s = simulation.add.sprite(150,150,'car');
	s.anchor.setTo(0.5,0.5);
	s.inputEnabled = true;
	
	
	simulation.add.sprite(200,100,'car1');
	simulation.add.sprite(100,200,'car2');
	simulation.add.sprite(300,400,'car3');
	simulation.add.sprite(500,300,'car4');
	simulation.add.sprite(0,500,'car5');
	simulation.add.sprite(900,700,'car6');
	simulation.add.sprite(600,600,'car7');
}

function update() {
	
	simulation.physics.enable(s, Phaser.Physics.ARCADE);
	
	isOver = s.input.pointerOver()? true : false;

    if (simulation.input.keyboard.isDown(Phaser.Keyboard.LEFT))
    {
        s.x -= 1;
    }
    else if (simulation.input.keyboard.isDown(Phaser.Keyboard.RIGHT))
    {
        s.x += 1;
    }

    if (simulation.input.keyboard.isDown(Phaser.Keyboard.UP))
    {
        s.y -= 1;
    }
    else if (simulation.input.keyboard.isDown(Phaser.Keyboard.DOWN))
    {
        s.y += 1;
    }
	else if (simulation.input.keyboard.isDown(Phaser.Keyboard.Q))
    {
        s.angle += 1;
    }
	else if (simulation.input.keyboard.isDown(Phaser.Keyboard.E))
    {
		s.angle -= 1;
    }
	else if (simulation.input.keyboard.isDown(Phaser.Keyboard.Z))
    {
		s.angle += 90 ;
    }
	else if (simulation.input.keyboard.isDown(Phaser.Keyboard.X))
    {
		s.angle -= 90;
    }
	else if (simulation.input.activePointer.leftButton.isDown){
		if (isOver){
			moveAround();
		}
	}
	
	s.body.velocity.x = 0;
	s.body.velocity.y = 0;
	

}

function moveAround(){
	simulation.physics.arcade.moveToXY(
	s, s.body.x + 100,
	s.body.y + 100,
	30
	);
}