const char P_js[] =  R"=====(
var xmlHttp=createXmlHttpObject();
function createXmlHttpObject(){
  if(window.XMLHttpRequest){
    xmlHttp=new XMLHttpRequest();
  }else{
    xmlHttp=new ActiveXObject('Microsoft.XMLHTTP');
  }
  return xmlHttp;
}

function load_system(){
  if(xmlHttp.readyState==0 || xmlHttp.readyState==4){
    xmlHttp.open('GET','/web/system.json',true);
    xmlHttp.send(null);
    xmlHttp.onload = function(e) {
      jsonResponse=JSON.parse(xmlHttp.responseText);
      loadBlock(xmlHttp.onload);
    }
  }
}

function load_brake(){
  if(xmlHttp.readyState==0 || xmlHttp.readyState==4){
    xmlHttp.open('GET','/web/brake.json',true);
    xmlHttp.send(null);
    xmlHttp.onload = function(e) {
      jsonResponse=JSON.parse(xmlHttp.responseText);
      loadBlock(xmlHttp.onload);
    }
  }
}

function load_wheel_calibration(){
  if(xmlHttp.readyState==0 || xmlHttp.readyState==4){
    xmlHttp.open('GET','/get/wheel_calibration.json',true);
    xmlHttp.send(null);
    xmlHttp.onload = function(e) {
      jsonResponse=JSON.parse(xmlHttp.responseText);
      loadBlock(xmlHttp.onload);
    }
  }
}

function load_pedal(){
  if(xmlHttp.readyState==0 || xmlHttp.readyState==4){
    xmlHttp.open('GET','/web/pedal.json',true);
    xmlHttp.send(null);
    xmlHttp.onload = function(e) {
      jsonResponse=JSON.parse(xmlHttp.responseText);
      loadBlock(xmlHttp.onload);
    }
  }
}

function load_stats(){
  if(xmlHttp.readyState==0 || xmlHttp.readyState==4){
    xmlHttp.open('GET','/web/stats.json',true);
    xmlHttp.send(null);
    xmlHttp.onload = function(e) {
      jsonResponse=JSON.parse(xmlHttp.responseText);
      loadBlock(xmlHttp.onload);
    }
  }
}

function update_status(){
  if(xmlHttp.readyState==0 || xmlHttp.readyState==4){
    xmlHttp.open('GET','/web/stats.json',true);
    xmlHttp.send(null);
    xmlHttp.onload = function(e) {
      jsonResponse=JSON.parse(xmlHttp.responseText);
      updBlock(xmlHttp.onload);
    }
  }
}

function loadBlock() {
  var data2 = JSON.parse(xmlHttp.responseText);
  data = document.getElementsByTagName('body')[0].innerHTML;
  var new_string;
  for (var key in data2) {
    new_string = data.replace(new RegExp('{{'+key+'}}', 'g'), data2[key]);
    data = new_string;
  }
  document.getElementsByTagName('body')[0].innerHTML = new_string;
  var inputs = document.getElementsByTagName("input");
  var selects = document.getElementsByTagName("select");
  for (var key in data2) {
    if(data2[key] == 'checked'){
       for (var i = 0; i < inputs.length; i++) {
         if (inputs[i].id === key) {
           inputs[i].checked = "true";
         }
       }
    }
    for (var i = 0; i < selects.length; i++) {
      if (selects[i].id === key) {
        document.getElementById(key).value = data2[key];
      }
    }
  }
}

function updBlock() {
  var data2 = JSON.parse(xmlHttp.responseText);
  for (var key in data2) {
    var input2 = document.getElementById(key);
    if (input2 != null) {
      input2.innerText = data2[key];
    }
  }
}

function val(id) {
  var v = document.getElementById(id).value;
  return v;
}

function val_sw(nameSwitch) {
  switchOn = document.getElementById(nameSwitch);
  if (switchOn.checked){
    return 1;
  }
  return 0;
}

function send_request(submit,server,reload = 0){
  request = new XMLHttpRequest();
  request.open("GET", server, true);
  request.send();
}

function wheel_updater() {
  let wheelIntervalId = setInterval(() => {
    let resisorPosition = document.getElementById("resisorPosition");
    let wheelRotation = document.getElementById("wheelRotation");
    let wheelTarget = document.getElementById("wheelTarget");

    fetch("/get/wheel_state", {
        method: "POST",
        body: JSON.stringify({
          userId: 1,
          title: "wheel state",
          completed: false
        }),
        headers: {
          "Content-type": "application/json; charset=UTF-8"
        }
      })
      .then((response) => response.json())
      .then((json) => {
        resisorPosition.setAttribute("value", json.resisorPosition);
        wheelRotation.setAttribute("value", json.wheelRotation);
        wheelTarget.setAttribute("value", json.wheelTarget);
      });

  }, 500);
}

function can_updater() {
  let canIntervalId = setInterval(() => {
    let carSpeed = document.getElementById("carSpeed");
    let carTacho = document.getElementById("carTacho");

    fetch("/get/can_state", {
        method: "POST",
        body: JSON.stringify({
          userId: 1,
          title: "can state",
          completed: false
        }),
        headers: {
          "Content-type": "application/json; charset=UTF-8"
        }
      })
      .then((response) => response.json())
      .then((json) => {
        carSpeed.setAttribute("value", json.carSpeed);
        carTacho.setAttribute("value", json.carTacho);
      });

  }, 500);
}

)=====";