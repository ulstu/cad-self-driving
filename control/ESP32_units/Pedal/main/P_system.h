const char P_system[] =  R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="../../favicon.ico">

    <title>Системные настройки</title>

    <link href="bootstrap.min.css" rel="stylesheet">
    <link href="theme.css" rel="stylesheet">
    <script type = "text/javascript" src = "/script.js"></script>
    <script type = "text/javascript">
      function save(submit) {
          server = "/set/system?dhcp=" + val_sw('dhcp');
          server += "&ip1=" + val('ip1') + "&ip2=" + val('ip2') + "&ip3=" + val('ip3') + "&ip4=" + val('ip4')
          server += "&ma1=" + val('ma1') + "&ma2=" + val('ma2') + "&ma3=" + val('ma3') + "&ma4=" + val('ma4')
          server += "&gw1=" + val('gw1') + "&gw2=" + val('gw2') + "&gw3=" + val('gw3') + "&gw4=" + val('gw4')
          send_request(submit,server);
      }
    </script>

  </head>

  <body onload = "load_system();">

    <!-- Fixed navbar -->
    <nav class="navbar navbar-inverse navbar-fixed-top">
      <div class="container">
        <div class="navbar-header">
          <button type="button" class="navbar-toggle collapsed" data-toggle="collapse" data-target="#navbar" aria-expanded="false" aria-controls="navbar">
            <span class="sr-only">Toggle navigation</span>
            <span class="icon-bar"></span>
            <span class="icon-bar"></span>
            <span class="icon-bar"></span>
          </button>
          <a class="navbar-brand" href="/">Газ/Руль</a>
        </div>
        <div id="navbar" class="navbar-collapse collapse">
          <ul class="nav navbar-nav">
            <li><a href="/">Усилитель руля</a></li>
            <li><a href="/pedal.html">Педаль Газа</a></li>
            <li><a href="/can.html">CAN Шина</a></li>
            <li class="active"><a href="/system.html">Система</a></li>
          </ul>
        </div>
      </div>
    </nav>

    <div class="container theme-showcase" role="main">
      <div class="page-header">
        <h1>Управление системой</h1>
      </div>
      <h3>Обновление прошивки</h3>
      <br>
      <table border="0" style="width:100%;">
          <tr>
              <td>
                  <label for="newfile">Файл обновления</label>
              </td>
              <td>
                  <input id="newfile" type="file" class="form-control-file">
              </td>
              <td>
                  <button id="upload" type="button" onclick="upload()" class="btn btn-primary">Отправить и перезагрузить</button>
              </td>
          </tr>
      </table>
      <br>
      <h3>IP адрес</h3>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td width="60%">Использовать DHCP</td>
            <td><input id="dhcp" type="checkbox"></td>
          </tr>
          <tr>
            <td>IP</td>
            <td>
              <input type="number" id="ip1" min="0" max="255" class="raz" value="{{ip1}}">.
              <input type="number" id="ip2" min="0" max="255" class="raz" value="{{ip2}}">.
              <input type="number" id="ip3" min="0" max="255" class="raz" value="{{ip3}}">.
              <input type="number" id="ip4" min="0" max="255" class="raz" value="{{ip4}}">
            </td>
          </tr>
          <tr>
            <td>Маска</td>
            <td>
              <input type="number" id="ma1" min="0" max="255" class="raz" value="{{ma1}}">.
              <input type="number" id="ma2" min="0" max="255" class="raz" value="{{ma2}}">.
              <input type="number" id="ma3" min="0" max="255" class="raz" value="{{ma3}}">.
              <input type="number" id="ma4" min="0" max="255" class="raz" value="{{ma4}}">
            </td>
          </tr>
          <tr>
            <td>Шлюз</td>
            <td>
              <input type="number" id="gw1" min="0" max="255" class="raz" value="{{gw1}}">.
              <input type="number" id="gw2" min="0" max="255" class="raz" value="{{gw2}}">.
              <input type="number" id="gw3" min="0" max="255" class="raz" value="{{gw3}}">.
              <input type="number" id="gw4" min="0" max="255" class="raz" value="{{gw4}}">
            </td>
          </tr>
        </tbody>
      </table>
      <br>
      <center>
        <button type="button" onclick="save(this);" class="btn btn-primary">Сохранить настройки</button>
      </center>
    </div>

    <script language="JavaScript">
      function upload() {
          var upload_path = "/upload";
          var fileInput = document.getElementById("newfile").files;

          if (fileInput.length == 0) {
              alert("Файл обновления не выбран!");
          } else {
              document.getElementById("newfile").disabled = true;
              document.getElementById("upload").disabled = true;

              var file = fileInput[0];
              var xhttp = new XMLHttpRequest();
              xhttp.onreadystatechange = function() {
                  if (xhttp.readyState == 4) {
                      if (xhttp.status == 200) {
                          document.open();
                          document.write(xhttp.responseText);
                          document.close();
                      } else if (xhttp.status == 0) {
                          alert("Сервер отклонил запрос");
                          UpdatePage(false);
                      } else {
                          alert(xhttp.status + " Ошибка!\n" + xhttp.responseText);
                          UpdatePage(false);
                      }
                  }
              };
              xhttp.open("POST", upload_path, true);
              xhttp.send(file);
          }
      }
      </script>
  </body>
</html>
)=====";