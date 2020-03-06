
function newQuat(q) {
    return new THREE.Quaternion(q['x'], q['y'], q['z'], q['w']);
}

function new3DVec(x, y, z) {
    return new THREE.Vector3(x, y, z);
}

function setQuat(q) {
    vectorQuaternion = q;
    vectorQuaternion.normalize();
    updateRotationAxis();
    updateVectorVisuals();
    //updateRotationInfo();
    renderer.render(scene, camera);
}


var viewQuat = 'QC';
function processQuatJSON(data) {
    for (var key in data) {
	if (data.hasOwnProperty(key)) {
	    $('#' + key).text(data[key]['s']);
	}
    }
    q = newQuat(data[viewQuat]);
    setQuat(q);
    updateButtons();
}

var buttons = {};
function updateButtons() {
    for (var btnK in buttons) {
	if (buttons.hasOwnProperty(btnK)) {
	    if (btnK == viewQuat) {
		buttons[btnK].attr('disabled', 'disabled');
	    } else {
		buttons[btnK].removeAttr('disabled');
	    }
	}
    }
}

$(function() {
    console.log("hello");
    setInterval(function() {
	$.getJSON( "/latestQuaternion.py", processQuatJSON);
    }, 100);
    var available = ['QC', 'QJ', 'QE'];
    available.forEach(function(v, i) {
	buttons[v] = $('input', $('#' + v).parent());
	buttons[v].click(function() {
	    viewQuat = v;
	});
    });
});
