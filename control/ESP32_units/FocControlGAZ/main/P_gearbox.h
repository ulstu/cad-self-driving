const char P_gearbox[] =  R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="../../favicon.ico">

    <title>Двигатель АКПП</title>

    <link href="bootstrap.min.css" rel="stylesheet">
    <link href="theme.css" rel="stylesheet">
    <script type = "text/javascript" src = "/script.js"></script>
    <script type = "text/javascript">
      function save(submit) {
          server = "/set/gearbox?calibrate=" + val_sw('calibrate') + "&inverseEncoder=" + val_sw('inverseEncoder');
          server += "&encoderAngle=" + val('encoderAngle') + "&voltageLimit=" + val('voltageLimit') + "&velLimitHard=" + val('velLimitHard');
          server += "&velProp=" + val('velProp') + "&velIntegral=" + val('velIntegral') + "&velDiff=" + val('velDiff');
          server += "&velRamp=" + val('velRamp') + "&velLimit=" + val('velLimit') + "&velFilter=" + val('velFilter');
          server += "&angleProp=" + val('angleProp') + "&angleLimit=" + val('angleLimit');
          send_request(submit,server);
      }
    </script>

  </head>

  <body onload = "load_gearbox();">

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
            <li><a href="/brake.html">Двигатель тормоза</a></li>
            <li class="active"><a href="#">Двигатель АКПП</a></li>
            <li><a href="/system.html">Система</a></li>
          </ul>
        </div>
      </div>
    </nav>

    <div class="container theme-showcase" role="main">
      <div class="page-header">
        <h1>Управление двигателем АКПП</h1>
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
          <tr>
            <td>Ограничение скорости</td>
            <td>
              <input type="number" step="1" id="velLimitHard" min="0.001" max="10000" value="{{velLimitHard}}">.
            </td>
          </tr>
        </tbody>
      </table>
      <br>
      <h3>Параметры ПИД регулятора скорости</h3>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td width="60%">Пропорциональный коэффициент</td>
            <td>
              <input type="number" step="0.1" id="velProp" min="0" max="1000" value="{{velProp}}">.
            </td>
          </tr>
          <tr>
            <td>Интегральный коэффициент</td>
            <td>
              <input type="number" step="1" id="velIntegral" min="0" max="1000" value="{{velIntegral}}">.
            </td>
          </tr>
          <tr>
            <td>Дифференциальный коэффициент</td>
            <td>
              <input type="number" step="0.001" id="velDiff" min="0" max="1" value="{{velDiff}}">.
            </td>
          </tr>
          <tr>
            <td>Рампа</td>
            <td>
              <input type="number" step="10" id="velRamp" min="0" max="10000" value="{{velRamp}}">.
            </td>
          </tr>
          <tr>
            <td>Ограничение</td>
            <td>
              <input type="number" step="1" id="velLimit" min="0" max="10000" value="{{velLimit}}">.
            </td>
          </tr>
          <tr>
            <td>Фильтрация</td>
            <td>
              <input type="number" step="0.001" id="velFilter" min="0" max="1" value="{{velFilter}}">.
            </td>
          </tr>
        </tbody>
      </table>
      <br>
      <h3>Параметры регулятора угла</h3>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td width="60%">Пропорциональный коэффициент</td>
            <td>
              <input type="number" step="0.1" id="angleProp" min="0" max="1000" value="{{angleProp}}">.
            </td>
          </tr>
          <tr>
            <td>Ограничение</td>
            <td>
              <input type="number" step="1" id="angleLimit" min="0" max="1000" value="{{angleLimit}}">.
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