import Draw from 'ol/interaction/Draw.js';
import {Feature, Map, Overlay, View} from 'ol/index.js';
import {Point} from 'ol/geom.js';
import {Circle, Fill, Stroke, Style, Icon} from 'ol/style.js';
import {OSM, Vector as VectorSource} from 'ol/source.js';
import GeoJSON from "ol/format/GeoJSON";
import {Tile as TileLayer, Vector as VectorLayer} from 'ol/layer.js';
import * as ol_color from 'ol/color';
import {useGeographic, fromLonLat} from 'ol/proj.js';
import {getCenter} from 'ol/extent.js';
import Translate from 'ol/interaction/Translate.js';

let init_point = [48.387626, 54.351436]
let current_map_file = null;

// AJAX-запрос для получения типов точек и добавления их в выпадающий список
$.get('/get_point_types', function(data){
  if (data['status'] != 'ok')
    return;
  data['pointtypes']['map-elements'].forEach(
    (element) => 
    $('<option/>', { value : element['ol-type']})
    .attr('data-color', element['color'])
    .attr('data-id', element['id'])
    .text(element['name'])
      .appendTo('#point-type')
  );
  addInteraction();
});

// AJAX-запрос для получения начальной точки
$.ajax({
  url: '/get_init_point',
  success: function(data){
      init_point = [data['lat'], data['lon']];
      if (data['mapfile'] != null) {
        current_map_file = data['mapfile'];
      }
  },
  async: false
});

useGeographic();

// Создание источника векторных слоёв и самого векторного слоя, на которых осуществляется добавление объектов карты
const raster = new TileLayer({source: new OSM()});
const source = new VectorSource({wrapX: false});
const vector = new VectorLayer({source: source});

const movePointSource = new VectorSource({wrapX: false});
const movePointLayer = new VectorLayer({
  source: movePointSource,
});
const translate = new Translate({
  source: movePointSource, 
});

// Создание объекта (feature) для отображения на карте
let ego_feature = new Feature({
  geometry: new Point(init_point),//(fromLonLat(init_point)),
  name: 'Ego vehicle',
});

// Стиль маркера для объекта автомобиля
const ego_marker_style = new Style({
  image: new Icon({
    anchor: [0.5, 46],
    anchorXUnits: 'fraction',
    anchorYUnits: 'pixels',
    src: 'https://cdn2.iconfinder.com/data/icons/top-view-cars-1/50/51-64.png'//'https://openlayers.org/en/latest/examples/data/icon.png'
  })
});

// Создание слоя для отображения объекта автомобиля
const ego_vehicle_layer = new VectorLayer({
  source: new VectorSource({
    wrapX: false,
    features: [ego_feature]
  }),
  style: ego_marker_style
});

// Инициализация карты с добавлением созданных слоёв
const map = new Map({
  layers: [raster, vector, ego_vehicle_layer, movePointLayer],
  target: 'map',
  view: new View({
    center: init_point,
    zoom: 17,
  }),
});

let draw; // global so we can remove it later
const typeSelect = document.getElementById('point-type');
const mapSelect = document.getElementById('maps');


// Функция для создания цвета с альфа-каналом
function colorWithAlpha(color, alpha) {
  const [r, g, b] = Array.from(ol_color.asArray(color));
  return ol_color.asString([r, g, b, alpha]);
}

// Функция для установки стиля рисования элемента редактора
function set_style(fill_color, stroke_color, stroke_width) {
  let style_stroke = new Stroke({
    color: stroke_color,
    width: 8
  });
  let style_fill = new Fill({
    color: colorWithAlpha(fill_color, 0.2)
  });
  let style_circle = new Circle ({
    radius: 8,
    fill: style_fill,
    stroke: style_stroke
  });

  var style = new Style({
    stroke: style_stroke,
    fill: style_fill,
    image: style_circle,
  });
  return style;
}

// Функция для добавления инструмента рисования на карту
function addInteraction() {
  const value = typeSelect.value;
  if (value !== 'None') {
    draw = new Draw({
      source: source,
      type: typeSelect.value,
      properties: []
    });

    var s = set_style('#' + typeSelect.options[typeSelect.selectedIndex].dataset.color,
      '#' + typeSelect.options[typeSelect.selectedIndex].dataset.color,
      0
    );

    draw.on('drawend', function(e) { 
      e.feature.setStyle(s);
      e.feature.setProperties({"id": typeSelect.options[typeSelect.selectedIndex].dataset.id}); 
    });
    map.addInteraction(draw);
  }
}

// отправка запроса на сохранение карты
function save_map_request(filename){
  var geojson  = new GeoJSON();
  var features = [];
  vector.getSource().getFeatures().forEach(function(feature) {
    feature.setProperties(feature.getStyle()) //add the layer styles to the feature as properties
    features.push(feature);
  });
  var json = geojson.writeFeatures(features);
  $.post('/save_map', {'filename': filename, 'map_data': json}, function(data) {
    if (data['status'] != 'ok'){
      alert(data['message']);
    }
    else 
      load_maps();
  });
}

// Сохранение карты. Если используемый файл определен, то сохранение происходит в этот файл, если нет, то 
// Выводится диалоговое окно с предложением ввести имя файла
function save_map(filename) {
  if (filename == null) {
    $("<div id='dynamic_dialog'><input name='map_filename' /></div>").dialog({
        modal: true,
        title:'Введите имя файлв:',
        buttons: {
          'OK': function () {
            var name = $('input[name="map_filename"]').val();
            current_map_file = name;
            save_map_request(current_map_file);
            $(this).dialog('close');
          },
          'Отмена': function () {
            $(this).dialog('close');
          }
        }
    });
  }
  else 
    save_map_request(current_map_file.replace(".geojson", ""));
}

// Функция для загрузки сохраненных ранее карт
function load_map(filename){
  $.post('/load_map', {'filename': filename}, function(data){
    if (data['status'] == 'ok'){
      source.clear();
      source.addFeatures(new GeoJSON().readFeatures(data['features']));
      current_map_file = filename.split('.')[0];

      vector.getSource().forEachFeature(function(feature) {
        var fill_color = null;
        var stroke_color = null;
        var stroke_width = null;
        var id = null;
        var v = feature.values_;
        if (typeof v==='object' && v!==null && !(v instanceof Array) && !(v instanceof Date)) {
          fill_color = feature.values_.fill_.color_;
          stroke_color = feature.values_.stroke_.color_;
          stroke_width = feature.values_.stroke_.width_;
          id = feature.values_.id;
        }
        else {
          fill_color = feature.values_[0].fill_.color_;
          stroke_color = feature.values_[0].stroke_.color_;
          stroke_width = feature.values_[0].stroke_.width_;
          id = feature.values_[0].id;
        }

        feature.setStyle(set_style(fill_color, stroke_color, stroke_width));
        feature.setProperties({"id": id}); 
      });
    }
    else
      alert(data['message']);
  });
}

// Метод загрузки списка ранее сохраненных карт и заполнение списка карт
function load_maps() {
  $.get('/get_maps', function(data){
    if (data['status'] != 'ok')
        return;
    $('#maps').empty();
    data['maps'].forEach(
      (element) => 
      $('<option/>', { value : element})
      .text(element)
      .appendTo('#maps')
    );
  });
}

// Установка обработчиков событий для элементов управления 
document.getElementById('undo').addEventListener('click', function () {
  draw.removeLastPoint();
});

typeSelect.onchange = function () {
  map.removeInteraction(draw);
  addInteraction();
};

document.getElementById('save').addEventListener('click', function () {
  save_map(current_map_file);
});

document.getElementById('saveas').addEventListener('click', function () {
  save_map(null);
});

document.getElementById('loadmap').addEventListener('click', function () {
  load_map(mapSelect.value);
});

let selected = null;
let isPointsMoveMode = false;
let isDeleteMode= false;

$('#changePointsCheckbox').change(function() {
  if($(this).is(":checked")) {
      isPointsMoveMode = true;
  }
  else {
    isPointsMoveMode = false;
    if (selected != null)
      // Удаление translate interaction после перетаскивания
      map.removeInteraction(translate);
      movePointSource.clear();
      selected = null;
  }
  console.log('Change points mode: ' + isPointsMoveMode);
});

$('#deletePointsCheckbox').change(function() {
  if($(this).is(":checked")) {
      isDeleteMode = true;
  }
  else {
    isDeleteMode = false;
    if (selected != null)
      // Удаление translate interaction после перетаскивания
      selected = null;
  }
  console.log('Change points mode: ' + isPointsMoveMode);
});

// Обработчик клика по карте для вывода координат в консоль и изменения объектов
map.on('click', function(event) {
  var coords = event.coordinate;
  console.log('Координаты клика:', coords, event.pixel);
  if (isDeleteMode) {
    map.forEachFeatureAtPixel(event.pixel, function(f, selLayer) {
      vector.getSource().removeFeature(f);
    });

  } 
  else if (isPointsMoveMode) {
    map.forEachFeatureAtPixel(event.pixel, function(f, selLayer) {
      selected = f;
      movePointSource.clear();

      let geometry = selected.getGeometry();
      let type = geometry.getType();
  
      let coordinates;
      if (type === 'Point') {
        coordinates = [geometry.getCoordinates()];
      } else if (type === 'LineString') {
        coordinates = geometry.getCoordinates();
      } else if (type === 'Polygon') {
        coordinates = geometry.getCoordinates()[0]; // только внешний контур
      } else 
        return;

      coordinates.forEach((coord, index) => {
        const pointFeature = new Feature(new Point(coord));
        pointFeature.set('index', index); // Сохраняем индекс точки
        movePointSource.addFeature(pointFeature);
      });

      map.addInteraction(translate);

      translate.on('translateend', function (evt) {
        // Обновляем только координаты перетаскиваемой точки
        evt.features.forEach((pfeature) => {
          const newIndex = pfeature.get('index');
          coordinates[newIndex] = pfeature.getGeometry().getCoordinates();
        });
  
        if (type === 'Polygon') {
          // Для Polygon обновляем внешний контур
          selected.getGeometry().setCoordinates([coordinates]);
        } else {
          selected.getGeometry().setCoordinates(coordinates);
        }
      });
    });
  }
});

// Загрузка карт и установка периодического обновления позиции маркера
load_maps();
if (current_map_file != null) {
  load_map(current_map_file);
  mapSelect.value = current_map_file;
}

// периодический запрос на обновление координат ТС
setInterval(
  () => $.get('/get_position', function(data){
    ego_feature.setGeometry(new Point([data['lat'], data['lon']]));
    ego_marker_style.getImage().setRotation(data['orientation']);
  }),
  2000,
);

setInterval(
  () => {
    console.log('image obj changed');
    var unique = $.now();
    $('#img_obj').attr('src', '/get_image?img_type=obj_detector&tm=' + unique);
  },
  700,
);

setInterval(
  () => {
    console.log('image seg changed');
    var unique = $.now();
    $('#img_seg').attr('src', '/get_image?img_type=seg&tm=' + unique);
  },
  700,
);