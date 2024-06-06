const char P_config[] =  R"=====(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="../../favicon.ico">

    <title>Настройки</title>

    <link href="bootstrap.min.css" rel="stylesheet">
    <link href="theme.css" rel="stylesheet">
    <script type = "text/javascript" src = "/script.js"></script>
    <script type = "text/javascript">
      function save(submit) {
          server = "/set/config?paLog=" + val_sw('paLog') + "&paNMin=" + val('paNMin') + "&paNMax=" + val('paNMax') + "&paDelta=" + val('paDelta');
          server += "&pbLog=" + val_sw('pbLog') + "&pbNMin=" + val('pbNMin') + "&pbNMax=" + val('pbNMax') + "&pbDelta=" + val('pbDelta');
          server += "&pcLog=" + val_sw('pcLog') + "&pcNMin=" + val('pcNMin') + "&pcNMax=" + val('pcNMax') + "&pcDelta=" + val('pcDelta');
          server += "&c1Log=" + val_sw('c1Log') + "&c1NMin=" + val('c1NMin') + "&c1NMax=" + val('c1NMax') + "&c1Delta=" + val('c1Delta');
          server += "&c2Log=" + val_sw('c2Log') + "&c2NMin=" + val('c2NMin') + "&c2NMax=" + val('c2NMax') + "&c2Delta=" + val('c2Delta');
          server += "&c3Log=" + val_sw('c3Log') + "&c3NMin=" + val('c3NMin') + "&c3NMax=" + val('c3NMax') + "&c3Delta=" + val('c3Delta');
          server += "&c4Log=" + val_sw('c4Log') + "&c4NMin=" + val('c4NMin') + "&c4NMax=" + val('c4NMax') + "&c4Delta=" + val('c4Delta');
          server += "&v1Log=" + val_sw('v1Log') + "&v1NMin=" + val('v1NMin') + "&v1NMax=" + val('v1NMax') + "&v1Delta=" + val('v1Delta');
          server += "&v2Log=" + val_sw('v2Log') + "&v2NMin=" + val('v2NMin') + "&v2NMax=" + val('v2NMax') + "&v2Delta=" + val('v2Delta');
          server += "&v3Log=" + val_sw('v3Log') + "&v3NMin=" + val('v3NMin') + "&v3NMax=" + val('v3NMax') + "&v3Delta=" + val('v3Delta');
          server += "&v4Log=" + val_sw('v4Log') + "&v4NMin=" + val('v4NMin') + "&v4NMax=" + val('v4NMax') + "&v4Delta=" + val('v4Delta');
          server += "&v5Log=" + val_sw('v5Log') + "&v5NMin=" + val('v5NMin') + "&v5NMax=" + val('v5NMax') + "&v5Delta=" + val('v5Delta');
          server += "&t1Log=" + val_sw('t1Log') + "&t1NMin=" + val('t1NMin') + "&t1NMax=" + val('t1NMax') + "&t1Delta=" + val('t1Delta');
          server += "&l1Log=" + val_sw('l1Log') + "&l1NMin=" + val('l1NMin') + "&l1NMax=" + val('l1NMax') + "&l1Delta=" + val('l1Delta');
          send_request(submit,server);
      }
    </script>
  </head>

  <body onload = "load_config();">

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
            <li><a href="outputs.html">Выходы</a></li>
            <li class="active"><a href="#">Настройки</a></li>
            <li><a href="system.html">Система</a></li>
          </ul>
        </div>
      </div>
    </nav>

    <div class="container theme-showcase" role="main">
      <div class="page-header">
        <h1>Установка параметров</h1>
      </div>
      <br>
      <table class="table">
        <thead>
          <tr>
            <th width="200dp">Напряжение AC</th>
            <th>Норма мин</th>
            <th>Норма макс</th>
            <th>Дельта</th>
            <th>Журналирование</th>
            <th>Действия</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>Фаза А</td>
            <td><input type="number" id="paNMin" min="0" max="500" value="{{paNMin}}"></td>
            <td><input type="number" id="paNMax" min="0" max="500" value="{{paNMax}}"></td>
            <td><input type="number" id="paDelta" min="1" max="500" value="{{paDelta}}"></td>
            <td><input id="paLog" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
          <tr>
            <td>Фаза B</td>
            <td><input type="number" id="pbNMin" min="0" max="500" value="{{pbNMin}}"></td>
            <td><input type="number" id="pbNMax" min="0" max="500" value="{{pbNMax}}"></td>
            <td><input type="number" id="pbDelta" min="1" max="500" value="{{pbDelta}}"></td>
            <td><input id="pbLog" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
          <tr>
            <td>Фаза C</td>
            <td><input type="number" id="pcNMin" min="0" max="500" value="{{pcNMin}}"></td>
            <td><input type="number" id="pcNMax" min="0" max="500" value="{{pcNMax}}"></td>
            <td><input type="number" id="pcDelta" min="1" max="500" value="{{pcDelta}}"></td>
            <td><input id="pcLog" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
        </tbody>
      </table>
      <br>
      <table class="table">
        <thead>
          <tr>
            <th width="200dp">Ток AC</th>
            <th>Норма мин</th>
            <th>Норма макс</th>
            <th>Дельта</th>
            <th>Журналирование</th>
            <th>Действия</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>Канал 1</td>
            <td><input type="number" id="c1NMin" min="0" max="500" value="{{c1NMin}}"></td>
            <td><input type="number" id="c1NMax" min="0" max="500" value="{{c1NMax}}"></td>
            <td><input type="number" id="c1Delta" min="1" max="500" value="{{c1Delta}}"></td>
            <td><input id="c1Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
          <tr>
            <td>Канал 2</td>
            <td><input type="number" id="c2NMin" min="0" max="500" value="{{c2NMin}}"></td>
            <td><input type="number" id="c2NMax" min="0" max="500" value="{{c2NMax}}"></td>
            <td><input type="number" id="c2Delta" min="1" max="500" value="{{c2Delta}}"></td>
            <td><input id="c2Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
          <tr>
            <td>Канал 3</td>
            <td><input type="number" id="c3NMin" min="0" max="500" value="{{c3NMin}}"></td>
            <td><input type="number" id="c3NMax" min="0" max="500" value="{{c3NMax}}"></td>
            <td><input type="number" id="c3Delta" min="1" max="500" value="{{c3Delta}}"></td>
            <td><input id="c3Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
          <tr>
            <td>Канал 4</td>
            <td><input type="number" id="c4NMin" min="0" max="500" value="{{c4NMin}}"></td>
            <td><input type="number" id="c4NMax" min="0" max="500" value="{{c4NMax}}"></td>
            <td><input type="number" id="c4Delta" min="1" max="500" value="{{c4Delta}}"></td>
            <td><input id="c4Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
        </tbody>
      </table>
      <br>
      <table class="table">
        <thead>
          <tr>
            <th width="200dp">Напряжение DC</th>
            <th>Норма мин</th>
            <th>Норма макс</th>
            <th>Дельта</th>
            <th>Журналирование</th>
            <th>Действия</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>Канал 1</td>
            <td><input type="number" id="v1NMin" min="0" max="500" value="{{v1NMin}}"></td>
            <td><input type="number" id="v1NMax" min="0" max="500" value="{{v1NMax}}"></td>
            <td><input type="number" id="v1Delta" min="1" max="500" value="{{v1Delta}}"></td>
            <td><input id="v1Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
          <tr>
            <td>Канал 2</td>
            <td><input type="number" id="v2NMin" min="0" max="500" value="{{v2NMin}}"></td>
            <td><input type="number" id="v2NMax" min="0" max="500" value="{{v2NMax}}"></td>
            <td><input type="number" id="v2Delta" min="1" max="500" value="{{v2Delta}}"></td>
            <td><input id="v2Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
          <tr>
            <td>Канал 3</td>
            <td><input type="number" id="v3NMin" min="0" max="500" value="{{v3NMin}}"></td>
            <td><input type="number" id="v3NMax" min="0" max="500" value="{{v3NMax}}"></td>
            <td><input type="number" id="v3Delta" min="1" max="500" value="{{v3Delta}}"></td>
            <td><input id="v3Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
          <tr>
            <td>Канал 4</td>
            <td><input type="number" id="v4NMin" min="0" max="500" value="{{v4NMin}}"></td>
            <td><input type="number" id="v4NMax" min="0" max="500" value="{{v4NMax}}"></td>
            <td><input type="number" id="v4Delta" min="1" max="500" value="{{v4Delta}}"></td>
            <td><input id="v4Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
          <tr>
            <td>Канал 5</td>
            <td><input type="number" id="v5NMin" min="0" max="500" value="{{v5NMin}}"></td>
            <td><input type="number" id="v5NMax" min="0" max="500" value="{{v5NMax}}"></td>
            <td><input type="number" id="v5Delta" min="1" max="500" value="{{v5Delta}}"></td>
            <td><input id="v5Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
        </tbody>
      </table>
      <br>
      <table class="table">
        <thead>
          <tr>
            <th width="200dp">Среда</th>
            <th>Норма мин</th>
            <th>Норма макс</th>
            <th>Дельта</th>
            <th>Журналирование</th>
            <th>Действия</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>Температура</td>
            <td><input type="number" id="t1NMin" min="0" max="500" value="{{t1NMin}}"></td>
            <td><input type="number" id="t1NMax" min="0" max="500" value="{{t1NMax}}"></td>
            <td><input type="number" id="t1Delta" min="1" max="500" value="{{t1Delta}}"></td>
            <td><input id="t1Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
          <tr>
            <td>Освещенность</td>
            <td><input type="number" id="l1NMin" min="0" max="500" value="{{l1NMin}}"></td>
            <td><input type="number" id="l1NMax" min="0" max="500" value="{{l1NMax}}"></td>
            <td><input type="number" id="l1Delta" min="1" max="500" value="{{l1Delta}}"></td>
            <td><input id="l1Log" type="checkbox"></td>
            <td>
              <button type="button" class="btn btn-xs btn-danger">Сбросить</button>
            </td>
          </tr>
        </tbody>
      </table>
      <br>
      <center>
        <button type="button" onclick="save(this);" class="btn btn-primary">Сохранить параметры</button>
      </center>
    </div> 

  </body>
</html>
)=====";