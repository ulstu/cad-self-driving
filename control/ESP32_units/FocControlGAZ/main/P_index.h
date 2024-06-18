const char P_index[] =  R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="../../favicon.ico">

    <title>Текущие показатели</title>

    <link href="bootstrap.min.css" rel="stylesheet">
    <link href="theme.css" rel="stylesheet">
    <script type = "text/javascript" src = "/script.js"></script>

  </head>

  <body onload = "load_stats();">

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
            <li class="active"><a href="#">Статус</a></li>
            <li><a href="/brake.html">Двигатель тормоза</a></li>
            <li><a href="/gearbox.html">Двигатель АКПП</a></li>
            <li><a href="/system.html">Система</a></li>
          </ul>
        </div>
      </div>
    </nav>

    <div class="container theme-showcase" role="main">
      <div class="page-header">
        <h1>Текущие показатели</h1>
      </div>
      <br>

      <h3>Тормоз</h3>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td width="60%">Двигатель включен</td>
            <td><input disabled id="brakeMotorEnable" type="checkbox"></td>
          </tr>
          <tr>
            <td>Угол поворота вала</td>
            <td>
              <input disabled type="number" id="brakeRotation" value="{{brakeRotation}}">.
            </td>
          </tr>
          <tr>
            <td>Целевое значение напряжения</td>
            <td>
              <input disabled type="number" id="brakeTarget" value="{{brakeTarget}}">.
            </td>
          </tr>
        </tbody>
      </table>
      <br>

      <h3>АКПП</h3>
      <br>
      <table class="table">
        <tbody>
          <tr>
            <td width="60%">Двигатель включен</td>
            <td><input disabled id="gearboxMotorEnable" type="checkbox"></td>
          </tr>
          <tr>
            <td>Угол поворота вала</td>
            <td>
              <input disabled type="number" id="gearboxRotation" value="{{gearboxRotation}}">.
            </td>
          </tr>
          <tr>
            <td>Целевое значение угла поворота</td>
            <td>
              <input disabled type="number" id="gearboxTarget" value="{{gearboxTarget}}">.
            </td>
          </tr>
        </tbody>
      </table>
      <br>
      

      <center>
        <button type="button" class="btn btn btn-info" onclick="update_status();">Обновить</button>
      </center>
    </div>

  </body>
</html>

)=====";