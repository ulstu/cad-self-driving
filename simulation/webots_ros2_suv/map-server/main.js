import Draw from 'ol/interaction/Draw.js';
import {Feature, Map, Overlay, View} from 'ol/index.js';
import {Point} from 'ol/geom.js';
import {Circle, Fill, Stroke, Style, Icon} from 'ol/style.js';
import {OSM, Vector as VectorSource} from 'ol/source.js';
import GeoJSON from "ol/format/GeoJSON";
import {Tile as TileLayer, Vector as VectorLayer} from 'ol/layer.js';
import * as ol_color from 'ol/color';
import {useGeographic, fromLonLat} from 'ol/proj.js';

let init_point = [48.387626, 54.351436]
let current_map_file = null;

$.ajax({
    url: '/get_init_point',
    success: function(data){
        init_point = [data['lat'], data['lon']]
    },
    async: false
});

useGeographic();

const raster = new TileLayer({
  source: new OSM(),
});

const source = new VectorSource({wrapX: false});

const vector = new VectorLayer({
  source: source,
});

let ego_feature = new Feature({
  geometry: new Point(init_point),//(fromLonLat(init_point)),
  name: 'Ego vehicle',
}); 

const ego_vehicle_layer = new VectorLayer({
  source: new VectorSource({
    wrapX: false,
    features: [ego_feature]
  }),
  style: new Style({
    image: new Icon({
      anchor: [0.5, 46],
      anchorXUnits: 'fraction',
      anchorYUnits: 'pixels',
      src: 'https://openlayers.org/en/latest/examples/data/icon.png'
    })
  })
});

const map = new Map({
  layers: [raster, vector, ego_vehicle_layer],
  target: 'map',
  view: new View({
    center: init_point,
    zoom: 17,
  }),
});

let draw; // global so we can remove it later
const typeSelect = document.getElementById('point-type');
const mapSelect = document.getElementById('maps');


function colorWithAlpha(color, alpha) {
  const [r, g, b] = Array.from(ol_color.asArray(color));
  return ol_color.asString([r, g, b, alpha]);
}

function set_style(fill_color, stroke_color, stroke_width) {
  let style_stroke = new Stroke({
    color: stroke_color,
    width: 2
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

document.getElementById('undo').addEventListener('click', function () {
  draw.removeLastPoint();
});

typeSelect.onchange = function () {
  map.removeInteraction(draw);
  addInteraction();
};

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

setTimeout(
  () => $.get('/get_position', function(data){
    console.log(data)
    ego_feature.geometry = new Point([data['lat'], data['lon']])
  }),
  500,
);

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
    save_map_request(current_map_file);
}

function isDict(v) {
  return typeof v==='object' && v!==null && !(v instanceof Array) && !(v instanceof Date);
}

function load_map(){
  $.post('/load_map', {'filename': mapSelect.value}, function(data){
    if (data['status'] == 'ok'){
      source.clear();
      source.addFeatures(new GeoJSON().readFeatures(data['features']));
      current_map_file = mapSelect.value.split('.')[0];

      vector.getSource().forEachFeature(function(feature) {
        var fill_color = null;
        var stroke_color = null;
        var stroke_width = null;
        var id = null;
        if (isDict(feature.values_)) {
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

document.getElementById('save').addEventListener('click', function () {
  save_map(current_map_file);
});

document.getElementById('saveas').addEventListener('click', function () {
  save_map(null);
});

document.getElementById('loadmap').addEventListener('click', function () {
  load_map(null);
});

load_maps();