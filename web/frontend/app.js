let state = {
  version: 0,
  status: "starting",
  task: "",
  step: 0,
  max_steps: 20,
  pose: [0, 0, 0],
  yaw: 0,
  collided: false,
  reasoning: "",
  reasoning_summary: "",
  scene_analysis: "",
  trajectory_queue: [],
  qwen_waypoints: [],
  trajectory_candidates: [],
  selected_trajectory: {},
  candidates: [],
  selected: [],
  error: "",
  model_name: "",
};

const camImg = document.getElementById("camImage");
const camPh = document.getElementById("camPlaceholder");
const downCamImg = document.getElementById("downCamImage");
const downCamPh = document.getElementById("downCamPlaceholder");
const sStep = document.getElementById("sStep");
const sStatus = document.getElementById("sStatus");
const sScene = document.getElementById("sScene");
const sReasoning = document.getElementById("sReasoning");
const sCandidates = document.getElementById("sCandidates");
const sTrajectory = document.getElementById("sTrajectory");
const sModel = document.getElementById("sModel");
const taskInput = document.getElementById("taskInput");
const taskBtn = document.getElementById("taskBtn");
const depthImg = document.getElementById("depthImage");
const depthPh = document.getElementById("depthPlaceholder");

depthImg.onload = function () {
  depthImg.style.display = "block";
  if (depthPh) depthPh.style.display = "none";
};
depthImg.onerror = function () {
  depthImg.style.display = "none";
  if (depthPh) {
    depthPh.style.display = "block";
    depthPh.textContent = "depth load failed";
  }
};

var es = new EventSource("/events");
es.onmessage = function (e) {
  try {
    var s = JSON.parse(e.data);
    state = s;
    updateUI(s);
  } catch (err) {
    console.warn(err);
  }
};

function updateTask() {
  var v = taskInput.value.trim();
  if (!v) return;
  taskBtn.disabled = true;
  fetch("/task", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ task: v }),
  })
    .then(function (r) {
      return r.json();
    })
    .then(function (d) {
      taskBtn.disabled = false;
      taskBtn.textContent = "OK";
      taskInput.value = d.task || taskInput.value;
      setTimeout(function () {
        taskBtn.textContent = "Update";
      }, 1500);
    })
    .catch(function () {
      taskBtn.disabled = false;
      taskBtn.textContent = "Error";
      setTimeout(function () {
        taskBtn.textContent = "Update";
      }, 1500);
    });
}

taskInput.addEventListener("keydown", function (e) {
  if (e.key === "Enter") updateTask();
});

function updateUI(s) {
  if (s.frame_version > 0) {
    camImg.src = "/frame?t=" + s.frame_version;
    camImg.style.display = "block";
    camPh.style.display = "none";
  }
  if (s.down_frame_version > 0) {
    downCamImg.src = "/down_frame?t=" + s.down_frame_version;
    downCamImg.style.display = "block";
    downCamPh.style.display = "none";
  }
  if (s.depth_version > 0) {
    depthImg.src = "/depth_frame?t=" + s.depth_version;
  } else if (depthPh) {
    depthPh.textContent = "depth waiting... bytes=" + (s.depth_bytes || 0);
  }

  sModel.textContent = s.model_name;
  if (!s.model_name) sModel.style.display = "none";
  sStep.textContent = s.step + "/" + s.max_steps;
  if (s.collided) {
    sStatus.textContent = "COLLISION";
    sStatus.className = "badge badge-error";
  } else {
    sStatus.textContent = s.status;
    sStatus.className = "badge badge-" + s.status;
  }

  if (sScene) sScene.textContent = s.scene_analysis || "-";
  var txt = s.reasoning_summary || s.reasoning || "";
  if (!txt) sReasoning.innerHTML = '<span class="ph-muted">thinking...</span>';
  else sReasoning.textContent = txt;
  if (sTrajectory) renderTrajectoryDebug(s);
  renderLegacyCandidates(s);
}

function renderLegacyCandidates(s) {
  if (!sCandidates) return;
  if (s.candidates && s.candidates.length > 0) {
    var selStr = JSON.stringify(s.selected || []);
    var items = [];
    for (var ci = 0; ci < s.candidates.length; ci++) {
      var c = s.candidates[ci];
      var d = c.delta || {};
      var actions = c.actions || [];
      var isSel = JSON.stringify(actions) === selStr;
      items.push(
        '<div class="cand-item' +
          (isSel ? " selected" : "") +
          '">' +
          (isSel ? '<span class="cand-select-badge">SELECTED</span>' : "") +
          '<div class="cand-actions">[' +
          actions.join(", ") +
          "]</div>" +
          '<div class="cand-delta">dx=' +
          num(d.dx) +
          " dy=" +
          num(d.dy) +
          " dz=" +
          num(d.dz) +
          " dphi=" +
          num(d.dphi) +
          "</div></div>"
      );
    }
    sCandidates.innerHTML = items.join("\n");
  } else {
    sCandidates.innerHTML = '<span class="ph-muted">waiting for candidates...</span>';
  }
}

function renderTrajectoryDebug(s) {
  var q = s.qwen_waypoints || [];
  var queue = s.trajectory_queue || [];
  var candidates = s.trajectory_candidates || [];
  var selected = s.selected_trajectory || {};
  var html = [];
  html.push(section("Qwen raw incremental", fmtWpList(q, 5)));

  if (candidates.length) {
    html.push('<div class="traj-section"><span>Perturbed candidates</span>');
    for (var i = 0; i < Math.min(candidates.length, 5); i++) {
      var c = candidates[i] || {};
      html.push(
        '<div class="traj-cand' +
          (i === 0 ? " selected" : "") +
          '"><b>#' +
          (i + 1) +
          "</b> score=" +
          num(c.pre_score) +
          " conf=" +
          num(c.confidence) +
          " " +
          escapeHtml(c.source || "") +
          "<code>" +
          escapeHtml(fmtWpList(c.waypoints || [], 5)) +
          "</code></div>"
      );
    }
    html.push("</div>");
  } else {
    html.push('<div class="traj-section"><span>Perturbed candidates</span><em>disabled or waiting</em></div>');
  }

  html.push(section("Selected", fmtWpList(selected.waypoints || [], 5)));
  html.push(section("Queue world", fmtWpList(queue, 8)));
  sTrajectory.innerHTML = html.join("");
}

function section(label, value) {
  return '<div class="traj-section"><span>' + escapeHtml(label) + "</span><code>" + escapeHtml(value) + "</code></div>";
}

function fmtWpList(list, maxN) {
  if (!list || !list.length) return "[]";
  var n = Math.min(list.length, maxN || 6);
  var out = [];
  for (var i = 0; i < n; i++) {
    var p = list[i] || [];
    out.push("[" + num(p[0]) + "," + num(p[1]) + "," + num(p[2]) + "]");
  }
  if (list.length > n) out.push("...+" + (list.length - n));
  return out.join(" ");
}

function num(v) {
  var x = Number(v);
  return Number.isFinite(x) ? x.toFixed(2) : "0.00";
}

function escapeHtml(v) {
  return String(v).replace(/[&<>"']/g, function (ch) {
    return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[ch];
  });
}
