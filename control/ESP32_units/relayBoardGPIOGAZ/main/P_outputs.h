const char P_outputs[] =  R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="../../favicon.ico">

    <title>Выходы</title>

    <link href="bootstrap.min.css" rel="stylesheet">
    <link href="theme.css" rel="stylesheet">
    <script type = "text/javascript" src = "/script.js"></script>
    <script type = "text/javascript">
      function save(submit) {
          server = "/set/outputs?o1Log=" + val_sw('o1Log') + "&o1Tim=" + val('o1Tim');
          server += "&o2Log=" + val_sw('o2Log') + "&o2Tim=" + val('o2Tim');
          server += "&o3Log=" + val_sw('o3Log') + "&o3Tim=" + val('o3Tim');
          server += "&o4Log=" + val_sw('o4Log') + "&o4Tim=" + val('o4Tim');
          server += "&o5Log=" + val_sw('o5Log') + "&o5Tim=" + val('o5Tim');
          send_request(submit,server);
      }
      function change(submit, id, state) {
          server = "/set/output?out=" + id + "&state=" + state;
          send_request(submit, server);
      }
    </script>

  </head>
  <body onload="load_outputs();">

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
          <a class="navbar-brand" href="#">Data collection system</a>
        </div>
        <div id="navbar" class="navbar-collapse collapse">
          <ul class="nav navbar-nav">
            <li><a href="/">Показатели</a></li>
            <li><a href="/eventlog.csv">События</a></li>
            <li class="active"><a href="#">Выходы</a></li>
            <li><a href="config.html">Настройки</a></li>
            <li><a href="system.html">Система</a></li>
          </ul>
        </div>
      </div>
    </nav>

    <div class="container theme-showcase" role="main">
      <div class="page-header">
        <h1>Управление выходами</h1>
      </div>
      <br>
      <table class="table">
        <thead>
          <tr>
            <th width="15%">Номер канала</th>
            <th width="15%">Состояние</th>
            <th width="15%">Таймер АПВ, с</th>
            <th width="15%">Время АПВ, с</th>
            <th width="15%">Журналирование</th>
            <th width="25%">Действия</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>Выход 1</td>
            <td><span id="o1Sta">{{o1Sta}}</span></td>
            <td><input type="number" id="o1Tim" min="1" max="65535" value="{{o1Tim}}"></td>
            <td><span id="o1APR">{{o1APR}}</span></td>
            <td><input id="o1Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-success" onclick="change(this, 1, 1)">Включить</button>
              <button type="button" class="btn btn-xs btn-warning" onclick="change(this, 1, 0)">Отключить до АПВ</button>
              <button type="button" class="btn btn-xs btn-danger" onclick="change(this, 1, 2)">Отключить</button>
            </td>
          </tr>
          <tr>
            <td>Выход 2</td>
            <td><span id="o2Sta">{{o2Sta}}</span></td>
            <td><input type="number" id="o2Tim" min="1" max="65535" value="{{o2Tim}}"></td>
            <td><span id="o2APR">{{o2APR}}</span></td>
            <td><input id="o2Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-success" onclick="change(this, 2, 1)">Включить</button>
              <button type="button" class="btn btn-xs btn-warning" onclick="change(this, 2, 0)">Отключить до АПВ</button>
              <button type="button" class="btn btn-xs btn-danger" onclick="change(this, 2, 2)">Отключить</button>
            </td>
          </tr>
          <tr>
            <td>Выход 3</td>
            <td><span id="o3Sta">{{o3Sta}}</span></td>
            <td><input type="number" id="o3Tim" min="1" max="65535" value="{{o3Tim}}"></td>
            <td><span id="o3APR">{{o3APR}}</span></td>
            <td><input id="o3Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-success" onclick="change(this, 3, 1)">Включить</button>
              <button type="button" class="btn btn-xs btn-warning" onclick="change(this, 3, 0)">Отключить до АПВ</button>
              <button type="button" class="btn btn-xs btn-danger" onclick="change(this, 3, 2)">Отключить</button>
            </td>
          </tr>
          <tr>
            <td>Выход 4</td>
            <td><span id="o4Sta">{{o4Sta}}</span></td>
            <td><input type="number" id="o4Tim" min="1" max="65535" value="{{o4Tim}}"></td>
            <td><span id="o4APR">{{o4APR}}</span></td>
            <td><input id="o4Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-success" onclick="change(this, 4, 1)">Включить</button>
              <button type="button" class="btn btn-xs btn-warning" onclick="change(this, 4, 0)">Отключить до АПВ</button>
              <button type="button" class="btn btn-xs btn-danger" onclick="change(this, 4, 2)">Отключить</button>
            </td>
          </tr>
          <tr>
            <td>ID led</td>
            <td><span id="o5Sta">{{o5Sta}}</span></td>
            <td><input type="number" id="o5Tim" min="1" max="65535" value="{{o5Tim}}"></td>
            <td><span id="o5APR">{{o5APR}}</span></td>
            <td><input id="o5Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-success" onclick="change(this, 5, 1)">Включить</button>
              <button type="button" class="btn btn-xs btn-warning" onclick="change(this, 5, 0)">Отключить до АПВ</button>
              <button type="button" class="btn btn-xs btn-danger" onclick="change(this, 5, 2)">Отключить</button>
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