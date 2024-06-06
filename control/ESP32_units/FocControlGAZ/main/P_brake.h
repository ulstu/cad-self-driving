const char P_brake[] =  R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="../../favicon.ico">

    <title>Двигатель тормоза</title>

    <link href="bootstrap.min.css" rel="stylesheet">
    <link href="theme.css" rel="stylesheet">
    <script type = "text/javascript" src = "/script.js"></script>
    <script type = "text/javascript">
      function save(submit) {
          server = "/set/brake?calibrate=" + val_sw('calibrate') + "&inverseEncoder=" + val_sw('inverseEncoder');
          server += "&encoderAngle=" + val('encoderAngle') + "&voltageLimit=" + val('voltageLimit');
          send_request(submit,server);
      }
    </script>

  </head>

  <body onload = "load_brake();">

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
          <a class="navbar-brand" href="#">Электропривод</a>
        </div>
        <div id="navbar" class="navbar-collapse collapse">
          <ul class="nav navbar-nav">
            <li><a href="/">Статус</a></li>
            <li class="active"><a href="#">Двигатель тормоза</a></li>
            <li><a href="/gearbox.html">Двигатель АКПП</a></li>
            <li><a href="/system.html">Система</a></li>
          </ul>
        </div>
      </div>
    </nav>

    <div class="container theme-showcase" role="main">
      <div class="page-header">
        <h1>Управление двигателем тормоза</h1>
      </div>
      <h3>Параметры двигателя (применяются при перезагрузке)</h3>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td width="60%">Калибровка при запуске</td>
            <td><input id="calibrate" type="checkbox"></td>
          </tr>
          <tr>
            <td>Инверсия энкодера</td>
            <td><input id="inverseEncoder" type="checkbox"></td>
          </tr>
          <tr>
            <td>Угол энкодера</td>
            <td>
              <input type="number" step="0.01" id="encoderAngle" min="0" max="7" value="{{encoderAngle}}">.
            </td>
          </tr>
          <tr>
            <td>Ограничение напряжения</td>
            <td>
              <input type="number" step="0.5" id="voltageLimit" min="0" max="12" value="{{voltageLimit}}">.
            </td>
          </tr>
        </tbody>
      </table>
      <br>
      <center>
        <button type="button" onclick="save(this);" class="btn btn-primary">Сохранить настройки</button>
      </center>
    </div>
  </body>
</html>
)=====";