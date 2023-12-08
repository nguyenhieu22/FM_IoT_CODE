const firebaseConfig = {
    apiKey: "AIzaSyAW7VIOL5CnPjLA98fDJ-J4yQFqyocabiQ",
    authDomain: "fm-iot-e0bdd.firebaseapp.com",
    databaseURL: "https://fm-iot-e0bdd-default-rtdb.asia-southeast1.firebasedatabase.app",
    projectId: "fm-iot-e0bdd",
    storageBucket: "fm-iot-e0bdd.appspot.com",
    messagingSenderId: "1047467017993",
    appId: "1:1047467017993:web:4cc232e93cef134bffb5d8"
  };
  
  firebase.initializeApp(firebaseConfig);
  firebase.analytics();
var freq_read = document.getElementById('value-frequency-read');
var nhietdofirebase = firebase.database().ref().child('FM_IoT/Frequency');
nhietdofirebase.on('value', snap=>freq_read.innerText = snap.val()); 

var freq_control = document.getElementById('value-frequency-control');
var nhietdofirebase = firebase.database().ref().child('FM_IoT/Frequency_control');
nhietdofirebase.on('value', snap=>freq_control.innerText = snap.val()); 

var volume_read = document.getElementById('read-volume');
var nhietdofirebase = firebase.database().ref().child('FM_IoT/Volume');
nhietdofirebase.on('value', snap=>volume_read.innerText = snap.val()); 

let btn0 = document.querySelector('#id-btn0');
btn0.addEventListener('click', func0 )

let btn1 = document.querySelector('#id-btn1');
btn1.addEventListener('click', func1 )

let btn2 = document.querySelector('#id-btn2');
btn2.addEventListener('click', func2 )

let btn3 = document.querySelector('#id-btn3');
btn3.addEventListener('click', func3 )

let btn4 = document.querySelector('#id-volume1');
btn4.addEventListener('click', func5 )
let btn5 = document.querySelector('#id-volume2');
btn5.addEventListener('click', func4 )

var number1 = 99.9;
var number2 = 0.1;
var number3 = 0.05;
var volume = 9 ;
document.querySelector('#volume-title').innerText = volume;
function func0(){
  number1 = (number1*10-number2*10)/10;
  if(number1<=87) number1 = 87;
  firebase.database().ref("/FM_IoT").update({"Frequency_control":number1});
}
function func1(){
  number1 = (number1*10-number3*10)/10;
  if(number1<=87) number1 = 87;
  firebase.database().ref("/FM_IoT").update({"Frequency_control":number1});
}
function func2(){
  number1 = (number1*10+number3*10)/10; 
  if(number1>=108) number1 = 108;
  firebase.database().ref("/FM_IoT").update({"Frequency_control":number1});
}
function func3(){
  number1 = (number1*10+number2*10)/10;
  if(number1>=108) number1 = 108;
  firebase.database().ref("/FM_IoT").update({"Frequency_control":number1});
}
function func4(){
  volume = volume +1;
  if(volume >=15) volume = 15;
  document.querySelector('#volume-title').innerText = volume;
  firebase.database().ref("/FM_IoT").update({"Volume_control":volume});
}
function func5(){
  volume = volume -1;
  if(volume <=0) volume = 0;
  document.querySelector('#volume-title').innerText = volume;
  firebase.database().ref("/FM_IoT").update({"Volume_control":volume});
}