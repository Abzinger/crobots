function testLayout(times, route) {
  start = new Date();
  for (var i = 0; i < times; i++) {
    var URL = '/Route/' + route + '/Layout/1';

    $.getJSON(URL, function (data) {
      var dataTest = data;
    });
  }

  var end = new Date();
  var testTime = Math.floor(1000 / (end - start));
  return testTime;

}

function testInstruction(times, route) {
  start = new Date();
  for (var i = 0; i < times; i++) {

    var URL = '/Route/' + route + '/Instructions';
    $.getJSON(URL, function (data) {
      routeInstructions = data;
    });
  }

  var end = new Date();
  var testTime = Math.floor(1000 / (end - start));
  return testTime;

}
