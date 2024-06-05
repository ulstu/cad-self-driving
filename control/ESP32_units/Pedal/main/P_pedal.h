const char P_pedal[] =  R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="../../favicon.ico">

    <title>Педаль газа</title>

    <link href="bootstrap.min.css" rel="stylesheet">
    <link href="theme.css" rel="stylesheet">
    <script type = "text/javascript" src = "/script.js"></script>
    <script type = "text/javascript">
      function save_pedal_config(submit) {
          server = "/set/pedal?lowPosition=" + val('lowPosition') + "&highPostition=" + val('highPostition');
          server += "&DACVoltage=" + val('DACVoltage');
          send_request(submit,server);
      }
    </script>

  </head>

  <body onload = "load_pedal();">

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
            <li class="active"><a href="/pedal.html">Педаль Газа</a></li>
            <li><a href="/can.html">CAN Шина</a></li>
            <li><a href="/system.html">Система</a></li>
          </ul>
        </div>
      </div>
    </nav>

    <div class="container theme-showcase" role="main">
      <h2>Текущие показатели</h2>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td width="60%">Положение первого резистора</td>
            <td>
              <input disabled type="number" id="resisorPosition">
            </td>
          </tr>
          <tr>
            <td>Положение второго резистора</td>
            <td>
              <input disabled type="number" id="wheelRotation">
            </td>
          </tr>
        </tbody>
      </table>
      <br>
      <h3>Настройка</h3>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td>Нижнее напряжение</td>
            <td>
              <input type="number" id="lowPosition" value="{{lowPosition}}">
            </td>
          </tr>
          <tr>
            <td>Верхнее напряжение</td>
            <td>
              <input type="number" id="highPostition" value="{{highPostition}}">
            </td>
          </tr>
          <tr>
            <td>Максимальное напряжение ЦАП</td>
            <td>
              <input type="number" id="DACVoltage" value="{{DACVoltage}}">
            </td>
          </tr>
        </tbody>
      </table>
      <br>
      <center>
        <button type="button" class="btn btn btn-info" onclick="save_pedal_config(this);">Сохранить</button>
      </center>
    </div>
  </body>
</html>
)=====";