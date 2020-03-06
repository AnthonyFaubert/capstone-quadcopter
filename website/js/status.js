
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
    updateRotationInfo();
    renderer.render(namespace.scene, namespace.camera);
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
}

var buttonQC, buttonQJ, buttonQE;
$(function() {
    console.log("hello");
    setInterval(function() {
	$.getJSON( "/latestQuaternion.py", processQuatJSON);
    }, 100);
    buttonQC = $('input', $('#QC').parent());
    buttonQJ = $('input', $('#QJ').parent());
    buttonQE = $('input', $('#QE').parent());
    buttonQC.click(function() {
	viewQuat = 'QC';
	buttonQC.attr('disabled', 'disabled');
	buttonQJ.removeAttr('disabled');
	buttonQE.removeAttr('disabled');
    });
    buttonQJ.click(function() {
	viewQuat = 'QJ';
	buttonQJ.attr('disabled', 'disabled');
	buttonQC.removeAttr('disabled');
	buttonQE.removeAttr('disabled');
    });
    buttonQE.click(function() {
	viewQuat = 'QE';
	buttonQE.attr('disabled', 'disabled');
	buttonQJ.removeAttr('disabled');
	buttonQC.removeAttr('disabled');
    });
});
