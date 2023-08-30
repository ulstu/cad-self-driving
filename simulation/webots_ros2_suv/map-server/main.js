import Draw from 'ol/interaction/Draw.js';
import {Feature, Map, Overlay, View} from 'ol/index.js';
import {Circle, Fill, Stroke, Style} from 'ol/style.js';
import {OSM, Vector as VectorSource} from 'ol/source.js';
import {Tile as TileLayer, Vector as VectorLayer} from 'ol/layer.js';
import * as ol_color from 'ol/color';
import {useGeographic} from 'ol/proj.js';

useGeographic();

const init_point = [48.387626, 54.351436];

const raster = new TileLayer({
  source: new OSM(),
});

const source = new VectorSource({wrapX: false});

const vector = new VectorLayer({
  source: source,
});

const map = new Map({
  layers: [raster, vector],
  target: 'map',
  view: new View({
    center: init_point,
    zoom: 9,
  }),
});

let draw; // global so we can remove it later
const typeSelect = document.getElementById('point-type');


function colorWithAlpha(color, alpha) {
  const [r, g, b] = Array.from(ol_color.asArray(color));
  return ol_color.asString([r, g, b, alpha]);
}

function addInteraction() {
  const value = typeSelect.value;
  if (value !== 'None') {
    draw = new Draw({
      source: source,
      type: typeSelect.value,
    });
    
    let style_stroke = new Stroke({
      color: '#' + typeSelect.options[typeSelect.selectedIndex].dataset.color,
      width: 2
    });
    let style_fill = new Fill({
      color: colorWithAlpha('#' + typeSelect.options[typeSelect.selectedIndex].dataset.color, 0.2)
    });
    let style_circle = new Circle ({
      radius: 8,
      fill: style_fill,
      stroke: style_stroke
    })

    var s = new Style({
      stroke: style_stroke, 
      fill: style_fill,
      image: style_circle,
    });
    draw.on('drawend', function(e) { e.feature.setStyle(s); });
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
      .text(element['name'])
      .appendTo('#point-type')
  );
  addInteraction();
});



