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
          <a class="navbar-brand" href="#">Data collection system</a>
        </div>
        <div id="navbar" class="navbar-collapse collapse">
          <ul class="nav navbar-nav">
            <li class="active"><a href="#">Показатели</a></li>
            <li><a href="/eventlog.csv">События</a></li>
            <li><a href="outputs.html">Выходы</a></li>
            <li><a href="config.html">Настройки</a></li>
            <li><a href="system.html">Система</a></li>
          </ul>
        </div>
      </div>
    </nav>

    <div class="container theme-showcase" role="main">
      <div class="page-header">
        <h1>Текущие показатели</h1>
        <button type="button" class="btn btn btn-info" onclick="update_status();">Обновить</button>
      </div>
      <br>

      <div class="row">
        <div class="col-md-12">
          <table class="table">
            <thead>
              <tr>
                <th width="15%">Напряжение AC</th>
                <th width="10%">Текущее</th>
                <th width="10%">Минимальное</th>
                <th width="10%">Максимальное</th>
                <th width="10%">Норма</th>
                <th width="15%">Ниже нормы</th>
                <th width="15%">Выше нормы</th>
                <th width="15%">Сброс статистики</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td>Фаза А</td>
                <td><span id="paVal">{{paVal}}</span></td>
                <td><span id="paMin">{{paMin}}</span></td>
                <td><span id="paMax">{{paMax}}</span></td>
                <td><span id="paNMin">{{paNMin}}</span>-<span id="paNMax">{{paNMax}}</span></td>
                <td><span id="paLess">{{paLess}}</span></td>
                <td><span id="paMore">{{paMore}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
              <tr>
                <td>Фаза B</td>
                <td><span id="pbVal">{{pbVal}}</span></td>
                <td><span id="pbMin">{{pbMin}}</span></td>
                <td><span id="pbMax">{{pbMax}}</span></td>
                <td><span id="pbNMin">{{pbNMin}}</span>-<span id="pbNMax">{{pbNMax}}</span></td>
                <td><span id="pbLess">{{pbLess}}</span></td>
                <td><span id="pbMore">{{pbMore}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
              <tr>
                <td>Фаза C</td>
                <td><span id="pcVal">{{pcVal}}</span></td>
                <td><span id="pcMin">{{pcMin}}</span></td>
                <td><span id="pcMax">{{pcMax}}</span></td>
                <td><span id="pcNMin">{{pcNMin}}</span>-<span id="pcNMax">{{pcNMax}}</span></td>
                <td><span id="pcLess">{{pcLess}}</span></td>
                <td><span id="pcMore">{{pcMore}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
      <br>
      <div class="row">
        <div class="col-md-12">
          <table class="table">
            <thead>
              <tr>
                <th width="15%">Ток AC</th>
                <th width="10%">Текущий</th>
                <th width="10%">Минимальный</th>
                <th width="10%">Максимальный</th>
                <th width="10%">Норма</th>
                <th width="15%">Ниже нормы</th>
                <th width="15%">Выше нормы</th>
                <th width="15%">Сброс статистики</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td>Канал 1</td>
                <td><span id="c1Val">{{c1Val}}</span></td>
                <td><span id="c1Min">{{c1Min}}</span></td>
                <td><span id="c1Max">{{c1Max}}</span></td>
                <td><span id="c1NMin">{{c1NMin}}</span>-<span id="c1NMax">{{c1NMax}}</span></td>
                <td><span id="c1Less">{{c1Less}}</span></td>
                <td><span id="c1More">{{c1More}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
              <tr>
                <td>Канал 2</td>
                <td><span id="c2Val">{{c2Val}}</span></td>
                <td><span id="c2Min">{{c2Min}}</span></td>
                <td><span id="c2Max">{{c2Max}}</span></td>
                <td><span id="c2NMin">{{c2NMin}}</span>-<span id="c2NMax">{{c2NMax}}</span></td>
                <td><span id="c2Less">{{c2Less}}</span></td>
                <td><span id="c2More">{{c2More}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
              <tr>
                <td>Канал 3</td>
                <td><span id="c3Val">{{c3Val}}</span></td>
                <td><span id="c3Min">{{c3Min}}</span></td>
                <td><span id="c3Max">{{c3Max}}</span></td>
                <td><span id="c3NMin">{{c3NMin}}</span>-<span id="c3NMax">{{c3NMax}}</span></td>
                <td><span id="c3Less">{{c3Less}}</span></td>
                <td><span id="c3More">{{c3More}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
              <tr>
                <td>Канал 4</td>
                <td><span id="c4Val">{{c4Val}}</span></td>
                <td><span id="c4Min">{{c4Min}}</span></td>
                <td><span id="c4Max">{{c4Max}}</span></td>
                <td><span id="c4NMin">{{c4NMin}}</span>-<span id="c4NMax">{{c4NMax}}</span></td>
                <td><span id="c4Less">{{c4Less}}</span></td>
                <td><span id="c4More">{{c4More}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
      <br>
      <div class="row">
        <div class="col-md-12">
          <table class="table">
            <thead>
              <tr>
                <th width="15%">Напряжение DC</th>
                <th width="10%">Текущee</th>
                <th width="10%">Минимальное</th>
                <th width="10%">Максимальное</th>
                <th width="10%">Норма</th>
                <th width="15%">Ниже нормы</th>
                <th width="15%">Выше нормы</th>
                <th width="15%">Сброс статистики</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td>Канал 1</td>
                <td><span id="v1Val">{{v1Val}}</span></td>
                <td><span id="v1Min">{{v1Min}}</span></td>
                <td><span id="v1Max">{{v1Max}}</span></td>
                <td><span id="v1NMin">{{v1NMin}}</span>-<span id="v1NMax">{{v1NMax}}</span></td>
                <td><span id="v1Less">{{v1Less}}</span></td>
                <td><span id="v1More">{{v1More}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
              <tr>
                <td>Канал 2</td>
                <td><span id="v2Val">{{v2Val}}</span></td>
                <td><span id="v2Min">{{v2Min}}</span></td>
                <td><span id="v2Max">{{v2Max}}</span></td>
                <td><span id="v2NMin">{{v2NMin}}</span>-<span id="v2NMax">{{v2NMax}}</span></td>
                <td><span id="v2Less">{{v2Less}}</span></td>
                <td><span id="v2More">{{v2More}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
              <tr>
                <td>Канал 3</td>
                <td><span id="v3Val">{{v3Val}}</span></td>
                <td><span id="v3Min">{{v3Min}}</span></td>
                <td><span id="v3Max">{{v3Max}}</span></td>
                <td><span id="v3NMin">{{v3NMin}}</span>-<span id="v3NMax">{{v3NMax}}</span></td>
                <td><span id="v3Less">{{v3Less}}</span></td>
                <td><span id="v3More">{{v3More}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
              <tr>
                <td>Канал 4</td>
                <td><span id="v4Val">{{v4Val}}</span></td>
                <td><span id="v4Min">{{v4Min}}</span></td>
                <td><span id="v4Max">{{v4Max}}</span></td>
                <td><span id="v4NMin">{{v4NMin}}</span>-<span id="v4NMax">{{v4NMax}}</span></td>
                <td><span id="v4Less">{{v4Less}}</span></td>
                <td><span id="v4More">{{v4More}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
              <tr>
                <td>Канал 5</td>
                <td><span id="v5Val">{{v5Val}}</span></td>
                <td><span id="v5Min">{{v5Min}}</span></td>
                <td><span id="v5Max">{{v5Max}}</span></td>
                <td><span id="v5NMin">{{v5NMin}}</span>-<span id="v5NMax">{{v5NMax}}</span></td>
                <td><span id="v5Less">{{v5Less}}</span></td>
                <td><span id="v5More">{{v5More}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
      <br>
      <div class="row">
        <div class="col-md-12">
          <table class="table">
            <thead>
              <tr>
                <th width="15%">Среда</th>
                <th width="10%">Текущее</th>
                <th width="10%">Минимальное</th>
                <th width="10%">Максимальное</th>
                <th width="10%">Норма</th>
                <th width="15%">Ниже нормы</th>
                <th width="15%">Выше нормы</th>
                <th width="15%">Сброс статистики</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td>Температура</td>
                <td><span id="t1Val">{{t1Val}}</span></td>
                <td><span id="t1Min">{{t1Min}}</span></td>
                <td><span id="t1Max">{{t1Max}}</span></td>
                <td><span id="t1NMin">{{t1NMin}}</span>-<span id="t1NMax">{{t1NMax}}</span></td>
                <td><span id="t1Less">{{t1Less}}</span></td>
                <td><span id="t1More">{{t1More}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
              <tr>
                <td>Освещенность</td>
                <td><span id="l1Val">{{l1Val}}</span></td>
                <td><span id="l1Min">{{l1Min}}</span></td>
                <td><span id="l1Max">{{l1Max}}</span></td>
                <td><span id="l1NMin">{{l1NMin}}</span>-<span id="l1NMax">{{l1NMax}}</span></td>
                <td><span id="l1Less">{{l1Less}}</span></td>
                <td><span id="l1More">{{l1More}}</span></td>
                <td><button type="button" class="btn btn-xs btn-danger">Сбросить</button></td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>


    </div>

  </body>
</html>

)=====";