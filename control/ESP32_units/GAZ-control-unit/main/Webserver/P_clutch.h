const char P_clutch[] =  R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="../../favicon.ico">

    <title>Актуатор сцепления</title>

    <link href="bootstrap.min.css" rel="stylesheet">
    <link href="theme.css" rel="stylesheet">
    <script type = "text/javascript" src = "/script.js"></script>
    <script type = "text/javascript">
      function save(submit) {
          server = "/set/clutch";
          server += "?disabledPos=" + val('disabledPos') + "&zeroPos=" + val('zeroPos') + "&fullEngPos=" + val('fullEngPos');
          server += "&minEngPos=" + val('minEngPos') + "&pressedPos=" + val('pressedPos') + "&curProp=" + val('curProp');
          server += "&curIntegral=" + val('curIntegral') + "&curDiff=" + val('curDiff') + "&curRamp=" + val('curRamp');
          server += "&curLimit=" + val('curLimit') + "&curFilter=" + val('curFilter') + "&velProp=" + val('velProp');
          server += "&velIntegral=" + val('velIntegral') + "&velDiff=" + val('velDiff') + "&velRamp=" + val('velRamp');
          server += "&velLimit=" + val('velLimit') + "&velFilter=" + val('velFilter') + "&posProp=" + val('posProp');
          server += "&posLimit=" + val('posLimit') + "&curOffset=" + val('curOffset');
          send_request(submit,server);
      }
    </script>
    <style>
      input[type='number']{
        width: 100px;
      } 
    </style>

  </head>

  <body onload = "load_clutch();">

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
          <a class="navbar-brand" href="#">Главный блок БПТС</a>
        </div>
        <div id="navbar" class="navbar-collapse collapse">
          <ul class="nav navbar-nav">
            <li><a href="/">Статус</a></li>
            <li><a href="/brake.html">Актуатор тормоза</a></li>
            <li class="active"><a href="#">Актуатор сцепления</a></li>
            <li><a href="/system.html">Система</a></li>
          </ul>
        </div>
      </div>
    </nav>

    <div class="container theme-showcase" role="main">
      <div class="page-header">
        <h1>Управление актуатором сцепленя</h1>
      </div>
      <h3>Параметры сцепления</h3>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td width="60%">Положение при отключении</td>
            <td>
              <input type="number" step="0.005" id="disabledPos" min="0" max="1" value="{{disabledPos}}">.
            </td>
          </tr>
          <tr>
            <td>Положение в отпущенном состоянии</td>
            <td>
              <input type="number" step="0.005" id="zeroPos" min="0" max="1" value="{{zeroPos}}">.
            </td>
          </tr>
          <tr>
            <td>Положение полного зацепления</td>
            <td>
              <input type="number" step="0.005" id="fullEngPos" min="0" max="1" value="{{fullEngPos}}">.
            </td>
          </tr>
          <tr>
            <td>Положение начала зацепления</td>
            <td>
              <input type="number" step="0.005" id="minEngPos" min="0" max="1" value="{{minEngPos}}">.
            </td>
          </tr>
          <tr>
            <td>Положение в нажатом состоянии</td>
            <td>
              <input type="number" step="0.005" id="pressedPos" min="0" max="1" value="{{pressedPos}}">.
            </td>
          </tr>
        </tbody>
      </table>
      <br>
      <h3>Параметры ПИД регулятора тока</h3>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td width="60%">Пропорциональный коэффициент</td>
            <td>
              <input type="number" step="0.1" id="curProp" min="0" max="10" value="{{curProp}}">.
            </td>
          </tr>
          <tr>
            <td>Интегральный коэффициент</td>
            <td>
              <input type="number" step="1" id="curIntegral" min="0" max="100" value="{{curIntegral}}">.
            </td>
          </tr>
          <tr>
            <td>Дифференциальный коэффициент</td>
            <td>
              <input type="number" step="0.001" id="curDiff" min="0" max="0.001" value="{{curDiff}}">.
            </td>
          </tr>
          <tr>
            <td>Рампа</td>
            <td>
              <input type="number" step="10" id="curRamp" min="0" max="1000000" value="{{curRamp}}">.
            </td>
          </tr>
          <tr>
            <td>Ограничение</td>
            <td>
              <input type="number" step="1" id="curLimit" min="0" max="1" value="{{curLimit}}">.
            </td>
          </tr>
          <tr>
            <td>Фильтрация</td>
            <td>
              <input type="number" step="0.001" id="curFilter" min="0" max="1" value="{{curFilter}}">.
            </td>
          </tr>
          <tr>
            <td>Смещение датчика тока</td>
            <td>
              <input type="number" step="1" id="curOffset" min="-1" max="3300" value="{{curOffset}}">.
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
      <h3>Параметры регулятора положения</h3>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td width="60%">Пропорциональный коэффициент</td>
            <td>
              <input type="number" step="0.1" id="posProp" min="0" max="1000" value="{{posProp}}">.
            </td>
          </tr>
          <tr>
            <td>Ограничение</td>
            <td>
              <input type="number" step="1" id="posLimit" min="0" max="1000" value="{{posLimit}}">.
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