const char P_index[] =  R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1">

    <title>Усилитель руля</title>

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

  <body onload = "load_wheel_calibration();wheel_updater();">

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
            <li class="active"><a href="/">Усилитель руля</a></li>
            <li><a href="/pedal.html">Педаль Газа</a></li>
            <li><a href="/can.html">CAN Шина</a></li>
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
              <input disabled type="number" id="resisorPosition">
            </td>
          </tr>
          <tr>
            <td>Угол поворота руля</td>
            <td>
              <input disabled type="number" id="wheelRotation">
            </td>
          </tr>
          <tr>
            <td>Целевое значение угла</td>
            <td>
              <input disabled type="number" id="wheelTarget">
            </td>
          </tr>
        </tbody>
      </table>
      <br>

      <h3>Калибровка</h3>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td>Левое положение</td>
            <td>
              <input type="number" id="wheelCalibrationLeft" value="{{wheelCalibrationLeft}}">
            </td>
          </tr>
          <tr>
            <td>Среднее положение</td>
            <td>
              <input type="number" id="wheelCalibrationCenter" value="{{wheelCalibrationCenter}}">
            </td>
          </tr>
          <tr>
            <td>Правое положение</td>
            <td>
              <input type="number" id="wheelCalibrationRight" value="{{wheelCalibrationRight}}">
            </td>
          </tr>
          <tr>
            <td>Полный угол поворота(от левого до правого положения)</td>
            <td>
              <input type="number" id="wheelCalibrationRange" value="{{wheelCalibrationRange}}">
            </td>
          </tr>
        </tbody>
      </table>

      <br>

      <h3>Параметры ПИД-регулятора</h3>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td>Коэффициент П</td>
            <td>
              <input type="number" id="position_P" value="{{position_P}}">
            </td>
          </tr>
          <tr>
            <td>Коэффициент И</td>
            <td>
              <input type="number" id="position_I" value="{{position_I}}">
            </td>
          </tr>
          <tr>
            <td>Коэффициент Д</td>
            <td>
              <input type="number" id="position_D" value="{{position_D}}">
            </td>
          </tr>
            <td>Рампа</td>
            <td>
              <input type="number" id="ramp" value="{{ramp}}">
            </td>
          </tr>
          <tr>
            <td>Лимит</td>
            <td>
              <input type="number" id="limit" value="{{limit}}">
            </td>
          </tr>
          <tr>
            <td>Фильтр</td>
            <td>
              <input type="number" id="filter" value="{{filter}}">
            </td>
          </tr>
        </tbody>
      </table>

      <br>

      <center>
        <button type="button" class="btn btn btn-info" onclick="save_wheel_calibration(this);">Сохранить</button>
      </center>
    </div>

  </body>
</html>

)=====";