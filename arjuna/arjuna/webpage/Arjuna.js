// --- 1. ROS CONNECTION ---
var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090' // Change this IP if controlling remotely
});

ros.on('connection', function() {
    document.getElementById("ros-status").innerHTML = '<i class="fa-solid fa-circle-check"></i> Connected';
    document.getElementById("ros-status").className = "status-indicator status-connected";
    addLog("INFO", "Connected to ROS 2 WebSocket server.");
    
    refreshTopics();
    setupBatteryMonitors(); 
    setupFireMonitor();
});

ros.on('error', function(error) {
    document.getElementById("ros-status").innerHTML = '<i class="fa-solid fa-triangle-exclamation"></i> Connection Error';
    document.getElementById("ros-status").className = "status-indicator status-disconnected";
    addLog("ERROR", "ROS connection error occurred.");
    resetBatteryDisplay();
});

ros.on('close', function() {
    document.getElementById("ros-status").innerHTML = '<i class="fa-solid fa-circle-xmark"></i> Disconnected';
    document.getElementById("ros-status").className = "status-indicator status-disconnected";
    addLog("WARN", "Disconnected from ROS 2 WebSocket server.");
    resetBatteryDisplay();
});

// --- 2. BATTERY & STATUS MONITORS ---
function setupBatteryMonitors() {
    var batTopic = new ROSLIB.Topic({ ros : ros, name : '/battery', messageType : 'std_msgs/Float32' });
    batTopic.subscribe(function(message) { document.getElementById("val-battery").innerText = Math.round(message.data) + "%"; });

    var voltTopic = new ROSLIB.Topic({ ros : ros, name : '/voltage', messageType : 'std_msgs/Float32' });
    voltTopic.subscribe(function(message) { document.getElementById("val-voltage").innerText = message.data.toFixed(1) + " V"; });

    var currTopic = new ROSLIB.Topic({ ros : ros, name : '/current', messageType : 'std_msgs/Float32' });
    currTopic.subscribe(function(message) { document.getElementById("val-current").innerText = message.data.toFixed(1) + " A"; });
}

// --- FIRE MONITOR FUNCTION ---
function setupFireMonitor() {
    var fireTopic = new ROSLIB.Topic({ 
        ros : ros, 
        name : '/fire', 
        messageType : 'std_msgs/Bool' 
    });

    var connectionHeader = document.querySelector('.connection-header');
    
    if (!document.getElementById("fire-indicator")) {
        var fireBadge = document.createElement("div");
        fireBadge.id = "fire-indicator";
        fireBadge.className = "status-indicator";
        fireBadge.style.display = "none"; 
        fireBadge.innerHTML = '<i class="fa-solid fa-fire"></i> FIRE DETECTED';
        connectionHeader.appendChild(fireBadge);
    }

    fireTopic.subscribe(function(message) {
        var fireBadge = document.getElementById("fire-indicator");
        var isFire = (message.data === true || message.data === 1); 

        if (isFire) {
            fireBadge.style.display = "inline-flex";           
            addLog("WARN", "🔥 ALERT: Fire source detected!");
        } else {
            fireBadge.style.display = "none";            
        }
    });
}

function resetBatteryDisplay() {
    document.getElementById("val-battery").innerText = "--";
    document.getElementById("val-voltage").innerText = "--";
    document.getElementById("val-current").innerText = "--";
}

// --- 3. DYNAMIC TOPIC LIST ---
var activeTopicListeners = []; 

function refreshTopics() {
    addLog("INFO", "Fetching active ROS 2 topic list...");
    
    activeTopicListeners.forEach(function(listener) { listener.unsubscribe(); });
    activeTopicListeners = [];

    ros.getTopics(function(result) {
        var topics = result.topics;
        var types = result.types;
        
        var camSelect = document.getElementById("camera-topic-select");
        camSelect.innerHTML = "";
        var defaultOpt = document.createElement('option');
        defaultOpt.value = "none";
        defaultOpt.innerHTML = "OFF";
        camSelect.appendChild(defaultOpt);

        var listContainer = document.getElementById("topic-container");
        listContainer.innerHTML = "";

        var combined = [];
        for (var i = 0; i < topics.length; i++) { combined.push({ 'name': topics[i], 'type': types[i] }); }
        combined.sort(function(a, b) { return a.name.localeCompare(b.name); });

        for (var i = 0; i < combined.length; i++) {
            let tName = combined[i].name;
            let tType = combined[i].type;

            var opt = document.createElement('option');
            opt.value = tName;
            opt.innerHTML = tName + " (" + tType + ")";
            camSelect.appendChild(opt);

            var rowDiv = document.createElement("div");
            rowDiv.className = "topic-row";
            var headerDiv = document.createElement("div");
            headerDiv.className = "topic-header";
            headerDiv.innerHTML = `<span class="topic-name">${tName}</span>`;
            
            var valSpan = document.createElement("span");
            valSpan.className = "topic-val";
            valSpan.id = "val-" + tName.replace(/\//g, "").replace(/_/g, "-"); 
            valSpan.innerText = "...";

            rowDiv.appendChild(headerDiv);
            rowDiv.appendChild(valSpan);
            listContainer.appendChild(rowDiv);

            var unsafeTypes = ['sensor_msgs/Image', 'sensor_msgs/CompressedImage', 'sensor_msgs/PointCloud2', 'sensor_msgs/LaserScan', 'nav_msgs/OccupancyGrid'];

            if (unsafeTypes.includes(tType)) {
                valSpan.innerText = "[Binary Data Stream]";
                valSpan.style.color = "var(--text-muted)";
            } else {
                subscribeToValue(tName, tType, valSpan.id);
            }
        }
        addLog("INFO", "Active ROS 2 topics loaded successfully.");
    });
}

function subscribeToValue(topicName, topicType, elementId) {
    var topicListener = new ROSLIB.Topic({ ros : ros, name : topicName, messageType : topicType });
    topicListener.subscribe(function(message) {
        var el = document.getElementById(elementId);
        if (el) {
            var msgStr = JSON.stringify(message);
            if (msgStr.length > 80) msgStr = msgStr.substring(0, 80) + "...";
            msgStr = msgStr.replace(/"/g, '').replace(/{/g, '').replace(/}/g, '');
            el.innerText = msgStr;
        }
    });
    activeTopicListeners.push(topicListener);
}

// --- 4. CAMERA FEED LOGIC ---
function updateCameraTopic() {
    var select = document.getElementById("camera-topic-select");
    var topic = select.value;
    var display = document.getElementById("camera-display");
    display.innerHTML = "";
    if (topic === "none") {
        display.innerHTML = '<p><i class="fa-solid fa-video-slash"></i> Camera Feed OFF</p>';
        return;
    }
    var host = window.location.hostname;
    if (!host) host = "localhost"; 
    var imgUrl = "http://" + host + ":8080/stream?topic=" + topic;
    var img = document.createElement("img");
    img.src = imgUrl; img.alt = "Camera Feed";
    img.onerror = function() { display.innerHTML = '<p style="color:var(--neon-red)"><i class="fa-solid fa-triangle-exclamation"></i> Error Loading Video Stream</p>'; };
    display.appendChild(img);
    addLog("INFO", "Switched camera stream to topic: " + topic);
}

// --- 5. MOVEMENT COMMANDS LOGIC ---
var cmdVel = new ROSLIB.Topic({ ros : ros, name : '/cmd_vel', messageType : 'geometry_msgs/Twist' });
var twist = new ROSLIB.Message({ linear : { x : 0.0, y : 0.0, z : 0.0 }, angular : { x : 0.0, y : 0.0, z : 0.0 } });
var cmdTimer = null;

function publishCmd() { cmdVel.publish(twist); }

function startMoving(direction) {
    if (cmdTimer) clearInterval(cmdTimer);
    let linSpeed = parseFloat(document.getElementById("linear-vel").value);
    let angSpeed = parseFloat(document.getElementById("angular-vel").value); 
    
    twist.linear.x = 0; twist.angular.z = 0;

    if (direction === 'up') { twist.linear.x = linSpeed; }
    else if (direction === 'down') { twist.linear.x = -linSpeed; }
    else if (direction === 'left') { twist.angular.z = angSpeed; }
    else if (direction === 'right') { twist.angular.z = -angSpeed; }

    addLog("INFO", "Moving: " + direction.toUpperCase());
    cmdTimer = setInterval(publishCmd, 100);
}

function stop() {
    if (cmdTimer) { clearInterval(cmdTimer); cmdTimer = null; }
    twist.linear.x = 0; twist.angular.z = 0; cmdVel.publish(twist);
    addLog("INFO", "Decelerating/Stopping robot.");
}

function emergencyStop() {
    stop(); 
    addLog("ERROR", "🚨 EMERGENCY STOP SIGNALLED"); 
}

function syncVal(type, value) {
    if (type === 'linear') { document.getElementById("linear-vel").value = value; document.getElementById("linear-box").value = value; } 
    else { document.getElementById("angular-vel").value = value; document.getElementById("angular-box").value = value; }
}

document.body.onkeyup = function(e) {
    if (e.key == " " || e.code == "Space" || e.keyCode == 32) { emergencyStop(); }
}



function toggleBtn(btn) { btn.classList.toggle("active"); }

// --- 6. VOICE TRIGGER LOGIC ---
var voiceTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/voice_trigger',
    messageType : 'std_msgs/Bool'
});

function triggerMic() {
    var btn = document.getElementById("btn-mic");
    
    // Visual feedback
    btn.classList.add("listening");
    addLog("INFO", "Microphone listening for voice command...");

    // Send trigger to Python script
    var msg = new ROSLIB.Message({ data : true });
    voiceTopic.publish(msg);

    // Remove animation after 5 seconds
    setTimeout(function() {
        btn.classList.remove("listening");
    }, 5000); 
}

// --- LOGGING HELPER ---
let logCount = 1;
function addLog(level, msg) {
    let table = document.getElementById("logTable").getElementsByTagName('tbody')[0];
    let row = table.insertRow(0); 
    let now = new Date();
    let timeString = now.getHours().toString().padStart(2, '0') + ":" + 
                     now.getMinutes().toString().padStart(2, '0') + ":" + 
                     now.getSeconds().toString().padStart(2, '0');
    row.innerHTML = `<td>${logCount++}</td><td>${timeString}</td><td style="color:${level === 'ERROR' ? 'var(--neon-red)' : level === 'WARN' ? '#ffa726' : 'var(--neon-green)'}; font-weight: bold;">${level}</td><td>${msg}</td>`;
}

function clearLogs() {
    document.getElementById("logTable").getElementsByTagName('tbody')[0].innerHTML = "";
    logCount = 1;
}

resetBatteryDisplay();
addLog("INFO", "Dashboard UI ready.");
