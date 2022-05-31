var lat = 0;
var lon = 0;
var velocity = 0;
var zoom = 18;
var map;
var markers;
var drawControl;
var lineLayer;
var markers;

var i = 0;
update = function () {
    var lonLat = new OpenLayers.LonLat(lon, lat).transform(new OpenLayers.Projection("EPSG:4326"), map.getProjectionObject());
    markers.addMarker(new OpenLayers.Marker(lonLat));
    if (i == 0)
        map.setCenter(lonLat, zoom);
    i++;
    map.updateSize();
};

save_point_type = function(lat, lon, point){
        $.getJSON("/setpoint?lat=" + lat + "&lon=" + lon + "&ptype=" + point, function(sdata){
            document.location.reload(true);
        });
        $('#pointTypeDialog').dialog("close");
}

draw_path = function() {
    $.getJSON('/getpath', function (data) {
        var points = new Array();
        var fromProjection = new OpenLayers.Projection("EPSG:4326");
        var toProjection   = new OpenLayers.Projection("EPSG:900913");

        for (k = 0; k < data.points.length; k++) {
            var lonLat = new OpenLayers.LonLat(data.points[k].lon, data.points[k].lat).transform(new OpenLayers.Projection("EPSG:4326"), map.getProjectionObject());
            var size = new OpenLayers.Size(15, 15);
            var marker_path = '/static/img/pathpoint.png';
            switch (data.points[k].ptype){
                case "start":
                    marker_path = '/static/img/pathstart.png';
                    break;
                case "finish":
                    marker_path = '/static/img/pathfinish.png';
                    break;
                case "return":
                    marker_path = '/static/img/pathreturn.png';
                    break;
            }
            var marker = new OpenLayers.Marker(lonLat, new OpenLayers.Icon(marker_path, size));
            marker.id = "path_" + k;
            markers.addMarker(marker);
            marker.events.register("click", marker, function(e){
                var savelonLat = new OpenLayers.LonLat(this.lonlat.lon, this.lonlat.lat).transform(map.getProjectionObject(), new OpenLayers.Projection("EPSG:4326"));
                $('#pointTypeDialog').dialog({
                    'title': 'Point types',
                    'buttons': {
                        'point': function(event) {
                            save_point_type(savelonLat.lat, savelonLat.lon, "point");
                        },
                        'start': function(event) {
                            save_point_type(savelonLat.lat, savelonLat.lon, "start");
                        },
                        'finish': function(event) {
                            save_point_type(savelonLat.lat, savelonLat.lon, "finish");
                        },
                        'return': function (event) {
                            save_point_type(savelonLat.lat, savelonLat.lon, "return");
                        },
                        'cancel': function(event){
                            $(this).dialog("close");
                        }
                    }
                });
                OpenLayers.Event.stop(e);
            });
            points.push((new OpenLayers.Geometry.Point(data.points[k].lon, data.points[k].lat)).transform(fromProjection, toProjection));
        }
        var vector = new OpenLayers.Layer.Vector("Path",
            {
                projection: "EPSG:4326",
                styleMap: new OpenLayers.StyleMap({
                    strokeColor: '#ff0000',
                    strokeOpacity: 0.8,
                    strokeWidth: 5
                })
            });
        vector.addFeatures([new OpenLayers.Feature.Vector(new OpenLayers.Geometry.LineString(points))]);
        map.addLayers([vector]);
    });
}

get_pos = function () {
    $.getJSON('/getpos', function (data) {
        lat = data.lat;
        lon = data.lon;
        velocity = data.velocity;
        $("#velocityspan").html(velocity);
        update();
        setTimeout(get_pos, 1000);
    })
};

get_dist = function(){
    $.getJSON('/getdist', function (data) {
        var state = JSON.stringify(data['cmd']).replace(new RegExp('"', 'g'), '');
        state += " | " + JSON.stringify(data['state']).replace(new RegExp('"', 'g'), '');
        $("#statuscontainer").html(state);
        $("#distcontainer").html(JSON.stringify(data['dist']));
        setTimeout(get_dist, 1000);
    })
}

get_carstate = function() {
    $.getJSON('/getcarstate', function (data) {
        console.log(data);
        var str = "RPM: " + data["rpm"] + "<br />";
        str += "TR: " + data["transmission"] + "<br />";
        str += "speed: " + data["velocity"] + "<br />";
        str += "wheel: " + data["wheel"] + "<br />";
        str += "throttle: " + data["throttle"] + "<br />";
        $("#carstatecontainer").html(str);
        setTimeout(get_carstate, 500);
    })

}

toggleDrawControl = function(element) {
    if ($("#drawroad").prop("checked"))
        drawControl.activate();
    else
        drawControl.deactivate()
}

function tile2long(x,z) {
    return (x / Math.pow(2, z) * 360 - 180);
}

function tile2lat(y,z) {
    var n = Math.PI - 2 * Math.PI * y / Math.pow(2, z);
    return (180 / Math.PI * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n))));
}

init = function () {
    map = new OpenLayers.Map("map", {
        controls: [
            new OpenLayers.Control.Navigation(),
            new OpenLayers.Control.PanZoomBar(),
            new OpenLayers.Control.Permalink(),
            new OpenLayers.Control.ScaleLine({geodesic: true}),
            new OpenLayers.Control.Permalink('permalink'),
            new OpenLayers.Control.MousePosition(),
            new OpenLayers.Control.Attribution()],
        maxExtent: new OpenLayers.Bounds(-20037508.34, -20037508.34, 20037508.34, 20037508.34),
        maxResolution: 156543.0339,
        numZoomLevels: 19,
        units: 'm',
        projection: new OpenLayers.Projection("EPSG:900913"),
        displayProjection: new OpenLayers.Projection("EPSG:4326")
    });

    layerMapnik = new OpenLayers.Layer.OSM.Mapnik("Mapnik");
    layerMapnik.setOpacity(0.4);
    map.addLayer(layerMapnik);

    var newLayer = new OpenLayers.Layer.OSM("Local Tiles", "/gettile?zoom=${z}&x=${x}&y=${y}", {
        numZoomLevels: 22,
        alpha: true,
        isBaseLayer: false, 
        crossOrigin: null
    });
    map.addLayer(newLayer);

    var switcherControl = new OpenLayers.Control.LayerSwitcher();
    map.addControl(switcherControl);
    switcherControl.maximizeControl();

    if (!map.getCenter()) {
        var lonLat = new OpenLayers.LonLat(lon, lat).transform(new OpenLayers.Projection("EPSG:4326"), map.getProjectionObject());
        markers = new OpenLayers.Layer.Markers("Markers");
        map.addLayer(markers);
        markers.addMarker(new OpenLayers.Marker(lonLat));
    }

    lineLayer = new OpenLayers.Layer.Vector("Added path", {
            projection: "EPSG:4326",
            styleMap: new OpenLayers.StyleMap({
                strokeColor: '#ff0000',
                strokeOpacity: 0.8,
                strokeWidth: 5
            })
        });
    map.addLayers([lineLayer]);

    drawControl = new OpenLayers.Control.DrawFeature(lineLayer, OpenLayers.Handler.Path);
    drawControl.events.register('featureadded', drawControl, function(evt) {
        var geom = evt.feature.geometry.transform('EPSG:3857', 'EPSG:4326');
        points = new Array();
        for (i = 0; i < geom.components.length; i++)
            points.push([(geom.components[i].y), (geom.components[i].x)])
        $.ajax({
            url: '/savepath',
            data: JSON.stringify(points),
            contentType: 'application/json; charset=utf-8',
            method: "POST",
            success: function(data){
                console.log("GPS points saved: " + data);
                document.location.reload(true);
            }
        });
    });
    map.addControl(drawControl);
    markers.setZIndex( 1005 );
    lineLayer.setZIndex( 1000 );
    draw_path();
    setTimeout(get_pos, 1000);
    setTimeout(get_dist, 1000);
    setTimeout(get_carstate, 500);
};

$(document).ready(function(){
    init();
    $("#drawroad").click(function (e) {
        toggleDrawControl(e);
    });
});
