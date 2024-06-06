const char P_can[] =  R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1">

    <title>CAN шина</title>

    <link href="bootstrap.min.css" rel="stylesheet">
    <link href="theme.css" rel="stylesheet">
    <script type = "text/javascript" src = "/script.js"></script>

    <script type = "text/javascript">
      function save_wheel_calibration(submit) {
          server = "/set/wheel_calibration?wheelCalibrationLeft=" + val('wheelCalibrationLeft') + "&wheelCalibrationCenter=" + val('wheelCalibrationCenter');
          server += "&wheelCalibrationRight=" + val('wheelCalibrationRight') + "&wheelCalibrationRange=" + val('wheelCalibrationRange');
          
          server += "&position_P=" + val('position_P') + "&position_I=" + val('position_I') + "&position_D=" + val('position_D');
          server += "&ramp=" + val('ramp') + "&limit=" + val('limit') + "&filter=" + val('filter');

          send_request(submit,server);
      }
    </script>
  </head>

  <body onload = "can_updater();">

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
            <li class="active"><a href="/can.html">CAN Шина</a></li>
            <li><a href="/system.html">Система</a></li>
          </ul>
        </div>
      </div>
    </nav>

    <div class="container theme-showcase" role="main">
      <div class="page-header">
        <h2>Текущие показатели</h2>
      </div>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td width="60%">Положение резистора</td>
            <td>
              <input disabled type="number" id="carSpeed">
            </td>
          </tr>
          <tr>
            <td>Угол поворота руля</td>
            <td>
              <input disabled type="number" id="carTacho">
            </td>
          </tr>
        </tbody>
      </table>
    </div>

  </body>
</html>

)=====";