#include <stdarg.h>
#include <ctype.h>

TelemetrySample session_samples[SESSION_SAMPLE_CAPACITY];
uint32_t session_sample_head = 0;
uint32_t session_sample_count = 0;
uint64_t session_total_samples = 0;
int64_t session_epoch_us = 0;
bool session_initialized = false;
bool session_setup_complete = false;
bool session_recording_frozen = false;
SessionSummary session_summary = {};
SessionSummary frozen_session_summary = {};
RobotState session_duration_state = ROBOT_FAILSAFE;
uint64_t session_duration_state_started_ms = 0;
bool session_rc_ever_healthy = false;
bool session_rc_fault_active = false;
bool session_sensor_status_initialized = false;
bool session_sensor_fault_active = false;
int8_t session_last_left_average = 0;
int8_t session_last_right_average = 0;
bool session_last_translation_seen = false;

char session_event_buffer[SESSION_EVENT_CAPACITY];
size_t session_event_head = 0;
size_t session_event_count = 0;
bool session_events_frozen = false;
portMUX_TYPE session_event_mux = portMUX_INITIALIZER_UNLOCKED;

WebServer diagnostic_server(80);
bool session_web_routes_registered = false;

const char SESSION_DASHBOARD_HTML[] PROGMEM = R"__MELTYHTML__(<!doctype html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Vibemelt Session Diagnostics</title>
<style>
:root{color-scheme:dark;font-family:system-ui;background:#0b1020;color:#eef2ff}body{margin:0;padding:14px;max-width:1100px;margin:auto}h1{font-size:1.45rem;margin:.25rem 0}h2{font-size:1.1rem;margin:.2rem 0 .8rem}.sub,.muted{color:#a5b4fc}.sub{margin:.2rem 0 1rem}.grid,.config-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:8px}.config-grid{grid-template-columns:repeat(auto-fit,minmax(245px,1fr))}.card,section,.setting{background:#151d35;border:1px solid #303b61;border-radius:10px;padding:10px}.setting{background:#10172b}.setting p{min-height:4.8em;font-size:.82rem;color:#c7d2fe}.label{color:#a5b4fc;font-size:.78rem}.value{font-size:1.1rem;font-weight:650;overflow-wrap:anywhere}section{margin-top:10px}a,button{display:inline-block;background:#4f46e5;color:white;border:0;border-radius:7px;padding:9px 11px;margin:3px;text-decoration:none}button.secondary{background:#334155}button:disabled{opacity:.45}input[type=number]{width:7em;background:#080d1b;color:#fff;border:1px solid #64748b;border-radius:6px;padding:8px}#config-status{min-height:1.2em;margin-top:6px;font-size:.85rem}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:.78rem}table{border-collapse:collapse;width:100%;font-size:.72rem}th,td{padding:5px;border-bottom:1px solid #303b61;text-align:right;white-space:nowrap}th:nth-child(2),td:nth-child(2){text-align:left}.scroll{overflow:auto}.error{color:#fca5a5}.success{color:#86efac}
.curve-panel{margin-top:16px;border-top:1px solid #303b61;padding-top:14px}.curve-panel p{font-size:.85rem}.curve-legend{display:flex;gap:18px;flex-wrap:wrap;font-size:.85rem;margin:8px 0}.curve-key{display:inline-block;width:28px;margin-right:6px;vertical-align:middle;border-top:3px solid #67e8f9}.curve-key.default{border-top:2px dotted #cbd5e1;opacity:.65}#spin-curve-svg{display:block;width:100%;height:auto;max-height:400px;background:#10172b;border-radius:8px}#spin-curve-status{min-height:1.3em;margin-top:6px;font-size:.85rem}input[aria-invalid=true]{border-color:#fca5a5}
.ramp-controls{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.ramp-controls .setting{padding:8px}.ramp-controls .value{display:block;font-size:1rem}.ramp-controls .setting p{min-height:0;margin:5px 0;font-size:.78rem}.ramp-controls button{padding:7px 9px}.ramp-controls input[type=number]{padding:6px;width:7em}@media(max-width:600px){.ramp-controls{grid-template-columns:1fr}.ramp-controls .setting{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:3px 8px;align-items:center}.ramp-controls .setting p{grid-column:1/-1;grid-row:2}.ramp-controls .setting .setting-actions{grid-column:1/-1;display:flex;gap:3px;align-items:center;flex-wrap:wrap}.ramp-controls .setting .setting-range{display:none}}
</style></head><body>
<h1>Vibemelt session diagnostics</h1><div class="sub">Local RAM log &middot; 10 Hz &middot; latest five minutes</div>
<div id="live" class="grid"></div>
  <section><h2>Drive and LED configuration</h2><div class="muted">Changes are saved to controller memory and take effect on the next drive after leaving WiFi mode.</div><div id="config" class="config-grid"></div><div id="config-status"></div><div class="curve-panel"><h2>Spin-up command ceiling</h2><p class="muted">Edit either ramp control to preview before saving. The graph shows logical command before the physical full-output limit.</p><div id="ramp-config" class="ramp-controls"></div><div class="curve-legend"><span><i class="curve-key"></i>Current preview</span><span><i class="curve-key default"></i>Firmware default</span></div><svg id="spin-curve-svg" viewBox="0 0 760 360" role="img" aria-label="Spin-up logical command ceiling versus body RPM"></svg><div id="spin-curve-status" class="muted" role="status" aria-live="polite">Loading ramp configuration...</div></div></section>
<section><div><a href="/api/timeline.csv">Download full CSV</a><a href="/api/melty.csv">Download MELTY CSV</a><a href="/api/events.txt">Download events</a></div><pre id="summary">Loading summary...</pre></section>
<section><div><button id="prev">Previous</button><button id="next">Next</button><label><input id="melty" type="checkbox"> Show MELTY rows only</label> <span id="page"></span></div><div class="muted">Throttle is raw receiver input (-100 to 100). Spin request, base, and ceiling use the logical 0–100% range; L/R output averages show final physical motor output. Tank drive bypasses the full-output percentage.</div><div class="scroll"><table><thead><tr><th>time</th><th>state</th><th>rpm</th><th>g</th><th>V</th><th>CH1</th><th>CH2</th><th>raw thr</th><th>spin% logical</th><th>base logical</th><th>ceil logical</th><th>L out%</th><th>R out%</th><th>var</th><th>flags</th></tr></thead><tbody id="rows"></tbody></table></div></section>
<script>
const live=document.querySelector('#live'),summary=document.querySelector('#summary'),rows=document.querySelector('#rows'),pageText=document.querySelector('#page'),configBox=document.querySelector('#config'),rampConfigBox=document.querySelector('#ramp-config'),configStatus=document.querySelector('#config-status'),spinCurveSvg=document.querySelector('#spin-curve-svg'),spinCurveStatus=document.querySelector('#spin-curve-status');let start=0,count=40,total=0,configBusy=false,savedConfiguration=null;
const card=(label,value)=>`<div class="card"><div class="label">${label}</div><div class="value">${value}</div></div>`;
const delayMs=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function fetchJson(url,options={}){const readOnly=!options.method||options.method==='GET',attempts=readOnly?3:1;let lastError;for(let attempt=1;attempt<=attempts;attempt++){const controller=new AbortController(),timeout=setTimeout(()=>controller.abort(),8000);try{const response=await fetch(url,{cache:'no-store',...options,signal:controller.signal});const text=await response.text();let result;try{result=JSON.parse(text)}catch(e){throw new Error(`Invalid JSON from ${url} (HTTP ${response.status})`)}if(!response.ok)throw new Error(result.error||`HTTP ${response.status}`);clearTimeout(timeout);return result}catch(e){clearTimeout(timeout);lastError=e;if(attempt<attempts)await delayMs(400)}}throw new Error(`${(lastError&&lastError.message)||'request failed'} after ${attempts} attempt${attempts===1?'':'s'}`)}
const configDefinitions=[
 {key:'translation_scale_factor',label:'Translation scale factor',help:'Scales how far each translation pulse moves a motor command away from the base spin command. 0.00 disables translation modulation; 1.00 applies the full pulse difference; 2.00 doubles that difference, subject to motor command limits.',min:0,max:2,step:.05,digits:2},
 {key:'percentdecel',label:'Translation pulse width (%)',help:'Live angular percentage of one rotation for each opposing pulse. The motors return to the base spin command immediately after the phase leaves each window. Larger values provide a longer impulse, but can increase braking, current draw, ESC stress, and motor heating.',min:1,max:50,step:1,digits:1},
 {key:'spin_ramp_start_rpm',label:'Ramp start RPM',help:'Body RPM at which the logical spin ceiling starts rising above its 200-command floor. This knee is independent of the ramp slope.',compactHelp:'Hold at command 200 until this body RPM.',min:0,max:5000,step:1,digits:2},
 {key:'acc_rate',label:'Ramp slope (coefficient)',help:'Controls how quickly the logical ceiling rises after the selected start RPM: 200 + coefficient × (RPM^1.2 − start RPM^1.2), capped at 999. Lower values rise more gently; changing the slope keeps the start RPM fixed.',compactHelp:'Higher rises faster after the start RPM; the start stays fixed.',min:.01,max:1,step:.01,digits:2},
 {key:'motor_full_scale_percent',label:'Full-output throttle (%)',help:'Sets both the spin-input endpoint and physical output limit for spin, translation and calibration. At 64.7%, 64.7% throttle requests 100% logical spin and physical output is capped at 64.7%. Full translation holds one motor at that cap and cuts the other. Tank drive bypasses this setting; held CH8 boost can reach full motor output. The spin acceleration limiter still applies.',min:1,max:100,step:.1,digits:2},
 {key:'top_led_phase_offset_deg',label:'Top LED phase offset (°)',help:'Additional display-only phase trim for the top heading marker, applied after the saved shared LED calibration. Positive values move the marker forward in phase. Motor timing is unchanged.',min:-180,max:180,step:1,digits:1},
 {key:'bottom_led_phase_offset_deg',label:'Bottom LED phase offset (°)',help:'Additional display-only phase trim for the bottom heading marker, applied after its normal 180° underside orientation. Motor timing is unchanged.',min:-180,max:180,step:1,digits:1}
];
function configurationValue(key){const input=document.querySelector(`#cfg-${key}`),definition=configDefinitions.find(d=>d.key===key),bounds=savedConfiguration&&savedConfiguration.limits&&savedConfiguration.limits[key];if(!input||!definition)return null;const value=Number(input.value),minimum=bounds?bounds[0]:definition.min,maximum=bounds?bounds[1]:definition.max,valid=input.value.trim()!==''&&Number.isFinite(value)&&value>=minimum&&value<=maximum;input.setAttribute('aria-invalid',valid?'false':'true');return valid?value:null}
function configurationDisplayValue(definition,value){return definition.key==='acc_rate'||definition.key==='spin_ramp_start_rpm'?String(Number(value)):Number(value).toFixed(definition.digits)}
function spinCurveCommand(rpm,startRpm,coefficient,model){if(!model||![rpm,startRpm,coefficient,model.floor_command,model.max_command,model.exponent].every(Number.isFinite))return NaN;const bodyRpm=Math.max(0,rpm),rise=Math.pow(Math.max(bodyRpm,startRpm),model.exponent)-Math.pow(startRpm,model.exponent);return Math.trunc(Math.max(model.floor_command,Math.min(model.max_command,model.floor_command+coefficient*rise)))}
function spinCurveLayout(){
 const width=Math.max(240,Math.round(spinCurveSvg.getBoundingClientRect().width||spinCurveSvg.clientWidth||760));
 return {width,height:320,left:60,right:width-18,top:24,bottom:270};
}
function spinCurvePath(startRpm,coefficient,model,layout=spinCurveLayout()){
 let path='';
 for(let rpm=0;rpm<=5000;rpm+=10){
  const x=layout.left+rpm/5000*(layout.right-layout.left);
  const y=layout.bottom-spinCurveCommand(rpm,startRpm,coefficient,model)/1000*(layout.bottom-layout.top);
  path+=`${rpm?'L':'M'}${x.toFixed(2)},${y.toFixed(2)} `;
 }
 return path;
}
function renderSpinCurve(){
 if(!savedConfiguration)return;
 const model=savedConfiguration.spin_curve,defaults=savedConfiguration.defaults;
 const start=configurationValue('spin_ramp_start_rpm'),coefficient=configurationValue('acc_rate');
 if(!model||!defaults||![model.floor_command,model.max_command,model.exponent,defaults.spin_ramp_start_rpm,defaults.acc_rate].every(Number.isFinite)){
  spinCurveSvg.innerHTML='';spinCurveStatus.className='error';
  spinCurveStatus.textContent='Ramp graph configuration is unavailable.';return;
 }
 const layout=spinCurveLayout(),xTicks=layout.width<460?[0,2500,5000]:[0,1000,2000,3000,4000,5000];
 spinCurveSvg.setAttribute('viewBox',`0 0 ${layout.width} ${layout.height}`);
 let axes='';
 for(const rpm of xTicks){
  const x=layout.left+rpm/5000*(layout.right-layout.left);
  axes+=`<line x1="${x}" y1="${layout.top}" x2="${x}" y2="${layout.bottom}" stroke="#303b61"/><text x="${x}" y="${layout.bottom+22}" text-anchor="middle">${rpm}</text>`;
 }
 for(let command=0;command<=1000;command+=200){
  const y=layout.bottom-command/1000*(layout.bottom-layout.top);
  axes+=`<line x1="${layout.left}" y1="${y}" x2="${layout.right}" y2="${y}" stroke="#303b61"/><text x="${layout.left-10}" y="${y+4}" text-anchor="end">${command}</text>`;
 }
 axes+=`<text x="${(layout.left+layout.right)/2}" y="${layout.height-5}" text-anchor="middle">Body RPM</text><text transform="translate(15 ${(layout.top+layout.bottom)/2}) rotate(-90)" text-anchor="middle">Logical command</text>`;
 const valid=start!==null&&coefficient!==null;
 const defaultPath=spinCurvePath(defaults.spin_ramp_start_rpm,defaults.acc_rate,model,layout);
 const previewPath=valid?spinCurvePath(start,coefficient,model,layout):null;
 spinCurveSvg.innerHTML=`<g fill="#c7d2fe" font-family="system-ui,sans-serif" font-size="12">${axes}</g>${valid?`<path id="spin-curve-preview" d="${previewPath}" fill="none" stroke="#67e8f9" stroke-width="4"/>`:''}<path id="spin-curve-default" d="${defaultPath}" fill="none" stroke="#cbd5e1" stroke-opacity=".65" stroke-width="2" stroke-dasharray="3 6"/>`;
 if(!valid){
  spinCurveStatus.className='error';
  spinCurveStatus.textContent='Enter a valid ramp start RPM and slope within their allowed ranges. The dotted default remains visible.';return;
 }
 const dirty=['spin_ramp_start_rpm','acc_rate'].some(key=>{
  return configurationValue(key)!==Number(savedConfiguration[key]);
 });
 spinCurveStatus.className=dirty?'success':'muted';
 spinCurveStatus.textContent=dirty?'Unsaved ramp preview. Save each changed control to apply this curve.':'Saved ramp curve. Edit either control to preview changes before saving.';
}
function isRampSetting(definition){return definition.key==='spin_ramp_start_rpm'||definition.key==='acc_rate'}
function renderConfigurationSetting(definition,configuration,compact=false){
 const d=definition,help=compact?d.compactHelp:d.help;
 return `<div class="setting"><label class="value" for="cfg-${d.key}">${d.label}</label><p>${help}</p><div class="setting-actions"><input id="cfg-${d.key}" aria-label="${d.label}" type="number" min="${d.min}" max="${d.max}" step="${d.step}" value="${configurationDisplayValue(d,configuration[d.key])}"><button onclick="updateConfiguration('${d.key}',false)">Save</button><button class="secondary" onclick="updateConfiguration('${d.key}',true)">Reset to ${Number(configuration.defaults[d.key]).toFixed(d.digits)}</button></div><div class="label setting-range">Allowed ${d.min} to ${d.max}</div></div>`;
}
function setConfigurationDisabled(disabled){[configBox,rampConfigBox].forEach(box=>box.querySelectorAll('button,input').forEach(control=>control.disabled=disabled))}
async function loadConfiguration(){
 try{
  const x=await fetchJson('/api/config.json');savedConfiguration=x;
  configBox.innerHTML=configDefinitions.filter(d=>!isRampSetting(d)).map(d=>renderConfigurationSetting(d,x)).join('');
  rampConfigBox.innerHTML=configDefinitions.filter(isRampSetting).map(d=>renderConfigurationSetting(d,x,true)).join('');
  renderSpinCurve();
 }catch(e){
  configBox.innerHTML=`<span class="error">Configuration load failed: ${e.message}</span>`;
  rampConfigBox.innerHTML='';spinCurveStatus.className='error';
  spinCurveStatus.textContent='Ramp graph could not load controller configuration.';
 }
}
async function updateConfiguration(name,reset){
 if(configBusy)return;
 if(reset&&!confirm('Reset this setting to its firmware default?'))return;
 const input=document.querySelector(`#cfg-${name}`),definition=configDefinitions.find(d=>d.key===name);
 if(!reset&&configurationValue(name)===null){
  configStatus.className='error';configStatus.textContent='Enter a finite value within the allowed range.';renderSpinCurve();return;
 }
 configBusy=true;setConfigurationDisabled(true);
 configStatus.className='';configStatus.textContent=reset?'Resetting...':'Saving...';
 try{
  let url=`${reset?'/api/config/reset':'/api/config'}?name=${encodeURIComponent(name)}`;
  if(!reset)url+=`&value=${encodeURIComponent(input.value)}`;
  const result=await fetchJson(url,{method:'POST'});
  savedConfiguration[name]=result.value;
  input.value=configurationDisplayValue(definition,result.value);input.setAttribute('aria-invalid','false');
  configStatus.className='success';configStatus.textContent=`${result.name} saved and verified as ${result.value}`;
  renderSpinCurve();
 }catch(e){configStatus.className='error';configStatus.textContent=e.message}
 finally{configBusy=false;setConfigurationDisabled(false)}
}
async function refreshLive(){try{const x=await fetchJson('/api/live.json');live.innerHTML=card('WiFi',x.wifi_active?'active':'inactive')+card('AP / clients',`${x.ap_ready?'ready':'recovering'} / ${x.ap_clients}`)+card('AP restarts / start failures',`${x.ap_restarts} / ${x.ap_start_failures}`)+card('RC link / loss',`${x.rc_link?'ready':'lost'} / ${x.rc_loss_seconds.toFixed(1)} s`)+card('State',x.robot_state)+card('Before WiFi',x.last_operating_state)+card('Recording',x.recording)+card('Samples',x.stored_samples)+card('RPM / g',`${x.rpm} / ${x.g_force.toFixed(2)}`)+card('Battery',`${x.battery_volts.toFixed(2)} V`)+card('Raw CH1 / CH2 / throttle',`${x.ch1} / ${x.ch2} / ${x.throttle}`)+card('Logical spin request (%)',x.spin_request_percent)+card('Logical base / ceiling (%)',`${x.base_spin_command} / ${x.acceleration_ceiling}`)+card('Full-output throttle (%)',x.motor_full_scale_percent.toFixed(2))+card('Physical base (%)',x.physical_base_spin_command)+card('Physical output L / R (%)',`${x.left_dshot_average} / ${x.right_dshot_average}`)+card('Limiter / safety gate',`${x.acceleration_limiter_active?'ON':'off'} / ${x.safety_gate_closed?'CLOSED':'open'}`)+card('Free / min heap',`${x.free_heap} / ${x.minimum_free_heap}`);return true}catch(e){live.innerHTML=`<span class="error">Live update failed: ${e.message}</span>`;return false}}
async function refreshSummary(){try{const x=await fetchJson('/api/summary.json');summary.textContent=JSON.stringify(x,null,2)}catch(e){summary.textContent=`Summary update failed: ${e.message}`}}
async function loadRows(){try{const x=await fetchJson(`/api/timeline.json?start=${start}&count=${count}`);total=x.total;const only=document.querySelector('#melty').checked;rows.innerHTML=x.rows.filter(r=>!only||r.state==='MELTY').map(r=>`<tr><td>${r.time_s.toFixed(2)}</td><td>${r.state}</td><td>${r.rpm}</td><td>${r.g_force.toFixed(2)}</td><td>${r.battery_volts.toFixed(2)}</td><td>${r.ch1}</td><td>${r.ch2}</td><td>${r.throttle}</td><td>${r.spin_request_percent}</td><td>${r.base_spin_command}</td><td>${r.acceleration_ceiling}</td><td>${r.left_dshot_average}</td><td>${r.right_dshot_average}</td><td>${r.rpm_variation_ewma}</td><td>0x${r.flags.toString(16).padStart(2,'0')}</td></tr>`).join('');pageText.textContent=`${start}-${Math.min(start+count,total)} of ${total}`;document.querySelector('#prev').disabled=start===0;document.querySelector('#next').disabled=start+count>=total}catch(e){pageText.innerHTML=`<span class="error">Timeline failed: ${e.message}</span>`}}
async function pollLive(){await refreshLive();setTimeout(pollLive,1000)}
async function initializeDashboard(){await loadConfiguration();await refreshLive();await refreshSummary();await loadRows();setTimeout(pollLive,1000)}
[configBox,rampConfigBox].forEach(box=>box.addEventListener('input',renderSpinCurve));window.addEventListener('resize',renderSpinCurve);document.querySelector('#prev').onclick=()=>{start=Math.max(0,start-count);loadRows()};document.querySelector('#next').onclick=()=>{start+=count;loadRows()};document.querySelector('#melty').onchange=loadRows;setTimeout(initializeDashboard,500);
</script></body></html>)__MELTYHTML__";

uint64_t session_elapsed_ms()
{
  if(!session_initialized) return 0;
  int64_t elapsed = esp_timer_get_time() - session_epoch_us;
  return elapsed > 0 ? (uint64_t)(elapsed / 1000) : 0;
}

int session_normalized_channel(int channel)
{
  return constrain(duty[channel] * 2 - 100, -100, 100);
}

int session_acceleration_ceiling(uint16_t sample_rpm)
{
  return constrain((int)lroundf(spin_acceleration_ceiling(sample_rpm) / 10.0f),
                   0, 100);
}

int session_spin_request_percent(float raw_throttle_percent)
{
  return constrain((int)lroundf(spin_requested_command(raw_throttle_percent) /
                                9.99f), 0, 100);
}

bool session_acceleration_limiter_active(uint16_t sample_rpm,
                                         float raw_throttle_percent)
{
  return spin_requested_command(raw_throttle_percent) >
           spin_acceleration_ceiling(sample_rpm);
}

SessionState session_state_for_robot(RobotState state)
{
  switch(state)
  {
    case ROBOT_ARMED_IDLE: return SESSION_DISARMED;
    case ROBOT_TANK: return SESSION_TANK;
    case ROBOT_SPINUP: return SESSION_SPINUP;
    case ROBOT_MELTY: return SESSION_MELTY;
    case ROBOT_CONFIG: return SESSION_CONFIG;
    case ROBOT_WIFI: return SESSION_WIFI;
    case ROBOT_FAILSAFE: return SESSION_FAILSAFE;
    case ROBOT_LOW_BATTERY: return SESSION_LOW_BATTERY;
    case ROBOT_SENSOR_FAULT: return SESSION_SENSOR_FAULT;
    default: return SESSION_FAILSAFE;
  }
}

SessionState session_state_code()
{
  return session_setup_complete ? session_state_for_robot(robot_state) : SESSION_BOOT;
}

const char *session_state_name(uint8_t state)
{
  static const char *names[] = {"BOOT", "DISARMED", "TANK", "SPINUP", "MELTY",
                                "CONFIG", "WIFI", "FAILSAFE", "LOW_BATTERY",
                                "SENSOR_FAULT"};
  return state < (sizeof(names) / sizeof(names[0])) ? names[state] : "UNKNOWN";
}

int8_t session_scaled_motor_average(int output)
{
  return (int8_t)constrain((int)lroundf(output * (100.0f / 999.0f)), -100, 100);
}

void session_insert_sample(const TelemetrySample &sample)
{
  session_samples[session_sample_head] = sample;
  session_sample_head = (session_sample_head + 1) % SESSION_SAMPLE_CAPACITY;
  if(session_sample_count < SESSION_SAMPLE_CAPACITY) session_sample_count++;
  session_total_samples++;
}

void session_update_summary(const TelemetrySample &sample, uint64_t sequence)
{
  if((sample.flags & SAMPLE_MISSED_OR_INVALID) != 0) return;
  if((sample.flags & SAMPLE_ACCEL_VALID) != 0)
  {
    if(sample.rpm > session_summary.maximum_rpm)
    {
      session_summary.maximum_rpm = sample.rpm;
      session_summary.maximum_rpm_sequence = sequence;
    }
    if(sample.g_centi_g > session_summary.maximum_g_centi_g)
    {
      session_summary.maximum_g_centi_g = sample.g_centi_g;
      session_summary.maximum_g_sequence = sequence;
    }
  }
  if((sample.flags & SAMPLE_BATTERY_VALID) != 0 &&
     (session_summary.minimum_battery_centivolts == 0 ||
      sample.battery_centivolts < session_summary.minimum_battery_centivolts))
  {
    session_summary.minimum_battery_centivolts = sample.battery_centivolts;
  }
}

void session_capture_current_sample(bool missed)
{
  TelemetrySample sample = {};
  sample.state = (uint8_t)session_state_code();
  if(missed)
  {
    sample.flags = SAMPLE_MISSED_OR_INVALID;
    session_insert_sample(sample);
    return;
  }

  int left_average = 0;
  int right_average = 0;
  bool translation_seen = false;
  snapshot_motor_output_interval(left_average, right_average, translation_seen);
  session_last_left_average = session_scaled_motor_average(left_average);
  session_last_right_average = session_scaled_motor_average(right_average);
  session_last_translation_seen = translation_seen;

  bool overflow = false;
  if(rpm < 0 || rpm > UINT16_MAX) overflow = true;
  sample.rpm = (uint16_t)constrain(rpm, 0, (int)UINT16_MAX);
  float g_centi = current_gforce * 100.0f;
  if(g_centi < 0.0f || g_centi > UINT16_MAX) overflow = true;
  sample.g_centi_g = (uint16_t)constrain((int)lroundf(g_centi), 0, (int)UINT16_MAX);
  float battery_centi = volts * 100.0f;
  if(battery_centi < 0.0f || battery_centi > UINT16_MAX) overflow = true;
  sample.battery_centivolts =
      (uint16_t)constrain((int)lroundf(battery_centi), 0, (int)UINT16_MAX);
  if(rpm_variation_ewma < 0.0f || rpm_variation_ewma > UINT16_MAX) overflow = true;
  sample.rpm_variation =
      (uint16_t)constrain((int)lroundf(rpm_variation_ewma), 0, (int)UINT16_MAX);
  sample.throttle_normalized = (int8_t)session_normalized_channel(CH_SPIN_THROTTLE);
  sample.ch1_normalized = (int8_t)session_normalized_channel(CH_TRANSLATE_LR);
  sample.ch2_normalized = (int8_t)session_normalized_channel(CH_TRANSLATE_FB);
  bool spin_state = robot_state == ROBOT_SPINUP || robot_state == ROBOT_MELTY ||
                    robot_state == ROBOT_CONFIG;
  sample.base_spin_command = spin_state
                               ? (int8_t)constrain((int)lroundf(spinspeed / 9.99f), -100, 100)
                               : 0;
  sample.left_dshot_average = session_last_left_average;
  sample.right_dshot_average = session_last_right_average;

  if(current_gforce_valid && accel_sensor_healthy) sample.flags |= SAMPLE_ACCEL_VALID;
  if(detected_battery_cells > 0 && volts > BATTERY_UNPLUGGED_THRESHOLD)
    sample.flags |= SAMPLE_BATTERY_VALID;
  if(rc_status) sample.flags |= SAMPLE_RC_VALID;
  if(overflow) sample.flags |= SAMPLE_MISSED_OR_INVALID;
  if(session_acceleration_limiter_active(sample.rpm, spin_throttle_input_percent))
    sample.flags |= SAMPLE_ACCEL_LIMITER_ACTIVE;
  if(!motor_output_gate_open()) sample.flags |= SAMPLE_MOTOR_SAFETY_GATE_CLOSED;
  if(translation_seen) sample.flags |= SAMPLE_TRANSLATION_PULSE_SEEN;
  if(!accel_sensor_healthy) sample.flags |= SAMPLE_SENSOR_FAULT_ACTIVE;

  uint64_t sequence = session_total_samples;
  session_insert_sample(sample);
  session_update_summary(sample, sequence);
}

void session_logger_begin()
{
  session_epoch_us = esp_timer_get_time();
  session_initialized = true;
  session_duration_state_started_ms = 0;
  session_summary.minimum_free_heap = ESP.getFreeHeap();
  session_capture_current_sample(false); // sequence zero: BOOT at 0.00 s
}

void service_session_logger()
{
  if(!session_initialized || session_recording_frozen) return;
  uint32_t free_heap = ESP.getFreeHeap();
  if(session_summary.minimum_free_heap == 0 || free_heap < session_summary.minimum_free_heap)
    session_summary.minimum_free_heap = free_heap;

  int64_t elapsed_us = esp_timer_get_time() - session_epoch_us;
  if(elapsed_us < 0) return;
  uint64_t latest_due_sequence = (uint64_t)elapsed_us / SESSION_SAMPLE_INTERVAL_US;
  while(session_total_samples <= latest_due_sequence)
  {
    bool missed = session_total_samples < latest_due_sequence;
    session_capture_current_sample(missed);
    if((session_total_samples & 0x3f) == 0) reset_loop_task_watchdog();
  }
}

void session_logger_delay(unsigned long delay_ms)
{
  unsigned long started = millis();
  while((millis() - started) < delay_ms)
  {
    service_session_logger();
    delay(1);
  }
}

void session_logger_mark_setup_complete()
{
  session_setup_complete = true;
}

void session_add_state_duration(SessionSummary &summary, RobotState state,
                                uint64_t elapsed_ms)
{
  if(state == ROBOT_TANK) summary.tank_ms += elapsed_ms;
  else if(state == ROBOT_SPINUP) summary.spinup_ms += elapsed_ms;
  else if(state == ROBOT_MELTY) summary.melty_ms += elapsed_ms;
}

void session_on_state_transition(int previous_state, int next_state)
{
  (void)previous_state;
  if(!session_initialized) return;
  uint64_t now_ms = session_elapsed_ms();
  session_add_state_duration(session_summary, session_duration_state,
                             now_ms - session_duration_state_started_ms);
  session_duration_state = (RobotState)next_state;
  session_duration_state_started_ms = now_ms;
}

SessionSummary session_summary_snapshot()
{
  SessionSummary result = session_summary;
  uint64_t now_ms = session_elapsed_ms();
  session_add_state_duration(result, session_duration_state,
                             now_ms - session_duration_state_started_ms);
  result.invalid_rc_frames = total_invalid_rc_frame_count;
  result.motor_scheduler_overruns = motor_scheduler_overrun_count;
  result.motor_scheduler_max_interval_us = motor_scheduler_max_interval_us;
  result.dshot_tx_api_errors = dshot_tx_error_count;
  uint32_t free_heap = ESP.getFreeHeap();
  if(result.minimum_free_heap == 0 || free_heap < result.minimum_free_heap)
    result.minimum_free_heap = free_heap;
  return result;
}

void session_logger_freeze()
{
  if(session_recording_frozen) return;
  service_session_logger();
  frozen_session_summary = session_summary_snapshot();
  session_recording_frozen = true;
  session_events_frozen = true;
}

void session_logger_resume()
{
  session_recording_frozen = false;
  session_events_frozen = false;
  service_session_logger(); // represents the hosted gap as missed fixed-time rows
}

void session_note_rc_status(bool healthy)
{
  if(healthy)
  {
    session_rc_ever_healthy = true;
    session_rc_fault_active = false;
  }
  else if(session_rc_ever_healthy && !session_rc_fault_active)
  {
    session_summary.failsafe_episodes++;
    session_rc_fault_active = true;
  }
}

void session_note_sensor_health(bool healthy)
{
  if(!session_sensor_status_initialized)
  {
    session_sensor_status_initialized = true;
    session_sensor_fault_active = !healthy;
    if(!healthy) session_summary.sensor_fault_episodes++;
    return;
  }
  if(healthy)
  {
    session_sensor_fault_active = false;
  }
  else if(!session_sensor_fault_active)
  {
    session_summary.sensor_fault_episodes++;
    session_sensor_fault_active = true;
  }
}

void session_append_event_text(const char *text)
{
  if(!session_initialized || session_events_frozen || text == nullptr) return;
  portENTER_CRITICAL(&session_event_mux);
  for(size_t i = 0; text[i] != '\0'; i++)
  {
    session_event_buffer[session_event_head] = text[i];
    session_event_head = (session_event_head + 1) % SESSION_EVENT_CAPACITY;
    if(session_event_count < SESSION_EVENT_CAPACITY) session_event_count++;
  }
  portEXIT_CRITICAL(&session_event_mux);
}

void session_log_event(const char *message)
{
  char line[320];
  double seconds = session_elapsed_ms() / 1000.0;
  snprintf(line, sizeof(line), "[MeltyBrain %.2fs] %s\n", seconds,
           message != nullptr ? message : "");
  session_append_event_text(line);
#ifdef ENABLE_SERIAL_DEBUG
  Serial.print(line);
#endif
}

void session_log_eventf(const char *format, ...)
{
  char message[256];
  va_list args;
  va_start(args, format);
  vsnprintf(message, sizeof(message), format, args);
  va_end(args);
  session_log_event(message);
}

uint32_t session_oldest_slot()
{
  return (session_sample_head + SESSION_SAMPLE_CAPACITY - session_sample_count) %
         SESSION_SAMPLE_CAPACITY;
}

const TelemetrySample &session_sample_at(uint32_t logical_index)
{
  uint32_t slot = (session_oldest_slot() + logical_index) % SESSION_SAMPLE_CAPACITY;
  return session_samples[slot];
}

uint64_t session_sequence_at(uint32_t logical_index)
{
  return session_total_samples - session_sample_count + logical_index;
}

void session_compute_window_summary(WindowSummary &window)
{
  memset(&window, 0, sizeof(window));
  for(uint32_t i = 0; i < session_sample_count; i++)
  {
    const TelemetrySample &sample = session_sample_at(i);
    if((sample.flags & SAMPLE_MISSED_OR_INVALID) == 0 &&
       (sample.flags & SAMPLE_ACCEL_VALID) != 0)
    {
      if(sample.rpm > window.maximum_rpm)
      {
        window.maximum_rpm = sample.rpm;
        window.maximum_rpm_sequence = session_sequence_at(i);
      }
      if(sample.g_centi_g > window.maximum_g_centi_g)
      {
        window.maximum_g_centi_g = sample.g_centi_g;
        window.maximum_g_sequence = session_sequence_at(i);
      }
    }
    if((sample.flags & SAMPLE_BATTERY_VALID) != 0 &&
       (window.minimum_battery_centivolts == 0 ||
        sample.battery_centivolts < window.minimum_battery_centivolts))
      window.minimum_battery_centivolts = sample.battery_centivolts;
    if(sample.state == SESSION_TANK) window.tank_samples++;
    else if(sample.state == SESSION_SPINUP) window.spinup_samples++;
    else if(sample.state == SESSION_MELTY) window.melty_samples++;
  }
}

void session_chunk_write(const char *data, size_t length)
{
  if(wifi_mode_active)
  {
    // iBus runs without its timer ISR. Service the UART between network chunks
    // so a large diagnostic response does not overflow the receiver buffer.
    update_channels();
  }
  diagnostic_server.chunkWrite(data, length);
}

void session_handle_timeline_csv(bool melty_only)
{
  diagnostic_server.sendHeader("Cache-Control", "no-store");
  diagnostic_server.sendHeader("Content-Disposition",
      melty_only ? "attachment; filename=vibemelt-melty.csv"
                 : "attachment; filename=vibemelt-timeline.csv");
  diagnostic_server.chunkResponseBegin("text/csv");
  const char *header = "time_s,state,rpm,g_force,battery_volts,ch1,ch2,throttle,spin_request_percent,base_spin_command,acceleration_ceiling,left_dshot_average,right_dshot_average,rpm_variation_ewma,limit_flags\n";
  session_chunk_write(header, strlen(header));
  char chunk[1200];
  size_t used = 0;
  char row[320];
  for(uint32_t i = 0; i < session_sample_count; i++)
  {
    const TelemetrySample &sample = session_sample_at(i);
    if(melty_only && sample.state != SESSION_MELTY) continue;
    uint64_t sequence = session_sequence_at(i);
    int spin_request = session_spin_request_percent(
                         (sample.throttle_normalized + 100) * 0.5f);
    int row_length = snprintf(row, sizeof(row),
        "%.2f,%s,%u,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%u,0x%02X\n",
        sequence * SESSION_SAMPLE_INTERVAL_SECONDS,
        session_state_name(sample.state), sample.rpm,
        sample.g_centi_g / 100.0, sample.battery_centivolts / 100.0,
        sample.ch1_normalized, sample.ch2_normalized, sample.throttle_normalized,
        spin_request, sample.base_spin_command,
        session_acceleration_ceiling(sample.rpm), sample.left_dshot_average,
        sample.right_dshot_average, sample.rpm_variation, sample.flags);
    if(row_length <= 0) continue;
    if(used + (size_t)row_length > sizeof(chunk))
    {
      session_chunk_write(chunk, used);
      used = 0;
      reset_loop_task_watchdog();
      yield();
    }
    memcpy(chunk + used, row, row_length);
    used += row_length;
  }
  if(used > 0) session_chunk_write(chunk, used);
  diagnostic_server.chunkResponseEnd();
}

void session_handle_live_json()
{
  char response[1800];
  int throttle = session_normalized_channel(CH_SPIN_THROTTLE);
  bool spin_state = robot_state == ROBOT_SPINUP || robot_state == ROBOT_MELTY ||
                    robot_state == ROBOT_CONFIG;
  int base_command = spin_state
                       ? constrain((int)lroundf(spinspeed / 9.99f), -100, 100) : 0;
  int physical_base_command = spin_state
                               ? session_scaled_motor_average(scale_motor_output(spinspeed, robot_state))
                               : 0;
  SessionSummary view = session_recording_frozen
                          ? frozen_session_summary : session_summary_snapshot();
  uint32_t esp_minimum_heap = ESP.getMinFreeHeap();
  if(view.minimum_free_heap == 0 || esp_minimum_heap < view.minimum_free_heap)
    view.minimum_free_heap = esp_minimum_heap;
  int current_left_average = session_scaled_motor_average(last_motor_left_command);
  int current_right_average = session_scaled_motor_average(last_motor_right_command);
  double wifi_rc_loss_seconds = wifi_rc_loss_since_ms == 0
                                  ? 0.0
                                  : (millis() - wifi_rc_loss_since_ms) / 1000.0;
  snprintf(response, sizeof(response),
      "{\"wifi_active\":%s,\"ap_ready\":%s,\"ap_clients\":%u,"
      "\"ap_restarts\":%lu,\"ap_start_failures\":%lu,\"rc_link\":%s,"
      "\"rc_loss_seconds\":%.2f,\"robot_state\":\"%s\",\"last_operating_state\":\"%s\"," 
      "\"rpm\":%d,\"g_force\":%.2f,\"battery_volts\":%.2f,\"ch1\":%d,\"ch2\":%d,"
      "\"throttle\":%d,\"spin_request_percent\":%d,\"base_spin_command\":%d,"
      "\"motor_full_scale_percent\":%.3f,\"physical_base_spin_command\":%d,"
      "\"acceleration_ceiling\":%d,\"left_dshot_average\":%d,\"right_dshot_average\":%d,"
      "\"acceleration_limiter_active\":%s,\"safety_gate_closed\":%s,\"free_heap\":%u,"
      "\"minimum_free_heap\":%u,\"stored_samples\":%u,\"elapsed_seconds\":%.2f,"
      "\"recording\":\"%s\",\"recording_complete\":%s}",
      wifi_mode_active ? "true" : "false",
      wifi_ap_services_ready ? "true" : "false",
      wifi_ap_services_ready ? WiFi.softAPgetStationNum() : 0,
      (unsigned long)wifi_ap_restart_count,
      (unsigned long)wifi_ap_start_failure_count,
      rc_status ? "true" : "false", wifi_rc_loss_seconds,
      session_state_name(session_state_for_robot(robot_state)),
      session_state_name(session_state_for_robot(last_operating_state)),
      rpm, current_gforce, volts,
      session_normalized_channel(CH_TRANSLATE_LR),
      session_normalized_channel(CH_TRANSLATE_FB), throttle,
      session_spin_request_percent(spin_throttle_input_percent),
      base_command, motor_full_scale_percent, physical_base_command,
      session_acceleration_ceiling(constrain(rpm, 0, UINT16_MAX)),
      current_left_average, current_right_average,
      session_acceleration_limiter_active(constrain(rpm, 0, UINT16_MAX),
                                           spin_throttle_input_percent)
        ? "true" : "false",
      motor_output_gate_open() ? "false" : "true", ESP.getFreeHeap(),
      view.minimum_free_heap, session_sample_count, session_elapsed_ms() / 1000.0,
      session_recording_frozen ? "frozen" : "active",
      (session_recording_frozen || session_sample_count == SESSION_SAMPLE_CAPACITY)
        ? "true" : "false");
  diagnostic_server.sendHeader("Cache-Control", "no-store");
  diagnostic_server.send(200, "application/json", response);
}

void session_handle_summary_json()
{
  SessionSummary view = session_recording_frozen
                          ? frozen_session_summary : session_summary_snapshot();
  uint32_t esp_minimum_heap = ESP.getMinFreeHeap();
  if(view.minimum_free_heap == 0 || esp_minimum_heap < view.minimum_free_heap)
    view.minimum_free_heap = esp_minimum_heap;
  WindowSummary window;
  session_compute_window_summary(window);
  char response[2300];
  snprintf(response, sizeof(response),
      "{\"since_boot\":{\"maximum_rpm\":%u,\"maximum_rpm_time_s\":%.2f,"
      "\"maximum_g_force\":%.2f,\"maximum_g_time_s\":%.2f,\"minimum_battery_volts\":%.2f,"
      "\"tank_time_s\":%.3f,\"spinup_time_s\":%.3f,\"melty_time_s\":%.3f,"
      "\"failsafe_episodes\":%u,\"sensor_fault_episodes\":%u,\"total_invalid_rc_frames\":%u,"
      "\"motor_scheduler_overruns\":%u,\"motor_scheduler_max_interval_us\":%u,"
      "\"esp32_dshot_tx_api_errors\":%u,\"minimum_free_heap\":%u},"
      "\"retained_window\":{\"samples\":%u,\"duration_s\":%.2f,\"maximum_rpm\":%u,"
      "\"maximum_rpm_time_s\":%.2f,\"maximum_g_force\":%.2f,\"maximum_g_time_s\":%.2f,"
      "\"minimum_battery_volts\":%.2f,\"tank_time_s\":%.2f,\"spinup_time_s\":%.2f,"
      "\"melty_time_s\":%.2f},\"notes\":{\"dshot\":\"ESP32 TX API status only; ESC reception is not acknowledged\","
      "\"storage\":\"volatile RAM; newest rows overwrite oldest after five minutes\"}}",
      view.maximum_rpm,
      view.maximum_rpm_sequence * SESSION_SAMPLE_INTERVAL_SECONDS,
      view.maximum_g_centi_g / 100.0,
      view.maximum_g_sequence * SESSION_SAMPLE_INTERVAL_SECONDS,
      view.minimum_battery_centivolts / 100.0, view.tank_ms / 1000.0,
      view.spinup_ms / 1000.0, view.melty_ms / 1000.0,
      view.failsafe_episodes, view.sensor_fault_episodes, view.invalid_rc_frames,
      view.motor_scheduler_overruns, view.motor_scheduler_max_interval_us,
      view.dshot_tx_api_errors, view.minimum_free_heap, session_sample_count,
      session_sample_count * SESSION_SAMPLE_INTERVAL_SECONDS, window.maximum_rpm,
      window.maximum_rpm_sequence * SESSION_SAMPLE_INTERVAL_SECONDS,
      window.maximum_g_centi_g / 100.0,
      window.maximum_g_sequence * SESSION_SAMPLE_INTERVAL_SECONDS,
      window.minimum_battery_centivolts / 100.0,
      window.tank_samples * SESSION_SAMPLE_INTERVAL_SECONDS,
      window.spinup_samples * SESSION_SAMPLE_INTERVAL_SECONDS,
      window.melty_samples * SESSION_SAMPLE_INTERVAL_SECONDS);
  diagnostic_server.sendHeader("Cache-Control", "no-store");
  diagnostic_server.send(200, "application/json", response);
}

void session_handle_timeline_json()
{
  uint32_t start = diagnostic_server.hasArg("start")
                     ? (uint32_t)max(0L, diagnostic_server.arg("start").toInt()) : 0;
  uint32_t count = diagnostic_server.hasArg("count")
                     ? (uint32_t)constrain(diagnostic_server.arg("count").toInt(), 1L, 500L)
                     : 200;
  start = min(start, session_sample_count);
  uint32_t end = min(start + count, session_sample_count);
  diagnostic_server.sendHeader("Cache-Control", "no-store");
  diagnostic_server.chunkResponseBegin("application/json");
  char buffer[480];
  int length = snprintf(buffer, sizeof(buffer),
                        "{\"start\":%u,\"count\":%u,\"total\":%u,\"rows\":[",
                        start, end - start, session_sample_count);
  session_chunk_write(buffer, length);
  for(uint32_t i = start; i < end; i++)
  {
    const TelemetrySample &sample = session_sample_at(i);
    uint64_t sequence = session_sequence_at(i);
    length = snprintf(buffer, sizeof(buffer),
        "%s{\"time_s\":%.2f,\"state\":\"%s\",\"rpm\":%u,\"g_force\":%.2f,"
        "\"battery_volts\":%.2f,\"ch1\":%d,\"ch2\":%d,\"throttle\":%d,"
        "\"spin_request_percent\":%d,\"base_spin_command\":%d,\"acceleration_ceiling\":%d,"
        "\"left_dshot_average\":%d,\"right_dshot_average\":%d,\"rpm_variation_ewma\":%u,\"flags\":%u}",
        i == start ? "" : ",", sequence * SESSION_SAMPLE_INTERVAL_SECONDS,
        session_state_name(sample.state),
        sample.rpm, sample.g_centi_g / 100.0, sample.battery_centivolts / 100.0,
        sample.ch1_normalized, sample.ch2_normalized, sample.throttle_normalized,
        session_spin_request_percent((sample.throttle_normalized + 100) * 0.5f),
        sample.base_spin_command,
        session_acceleration_ceiling(sample.rpm), sample.left_dshot_average,
        sample.right_dshot_average, sample.rpm_variation, sample.flags);
    session_chunk_write(buffer, length);
    if((i & 0x1f) == 0) reset_loop_task_watchdog();
  }
  session_chunk_write("]}", 2);
  diagnostic_server.chunkResponseEnd();
}

void session_handle_events_text()
{
  diagnostic_server.sendHeader("Cache-Control", "no-store");
  diagnostic_server.sendHeader("Content-Disposition",
                               "attachment; filename=vibemelt-events.txt");
  diagnostic_server.chunkResponseBegin("text/plain");
  char chunk[1024];
  size_t logical = 0;
  size_t oldest = (session_event_head + SESSION_EVENT_CAPACITY - session_event_count) %
                  SESSION_EVENT_CAPACITY;
  while(logical < session_event_count)
  {
    size_t amount = min(sizeof(chunk), session_event_count - logical);
    for(size_t i = 0; i < amount; i++)
      chunk[i] = session_event_buffer[(oldest + logical + i) % SESSION_EVENT_CAPACITY];
    session_chunk_write(chunk, amount);
    logical += amount;
    reset_loop_task_watchdog();
  }
  diagnostic_server.chunkResponseEnd();
}

bool drive_config_metadata(const String &name, float *&target,
                           const char *&nvs_key, float &default_value,
                           float &minimum_value, float &maximum_value)
{
  if(name == "translation_scale_factor")
  {
    target = &translation_scale_factor;
    nvs_key = "trans_scale";
    default_value = DEFAULT_TRANSLATION_SCALE_FACTOR;
    minimum_value = MIN_TRANSLATION_SCALE_FACTOR;
    maximum_value = MAX_TRANSLATION_SCALE_FACTOR;
    return true;
  }
  if(name == "percentdecel")
  {
    target = &percentdecel;
    nvs_key = "pulse_pct";
    default_value = DEFAULT_PERCENT_DECEL;
    minimum_value = MIN_PERCENT_DECEL;
    maximum_value = MAX_PERCENT_DECEL;
    return true;
  }
  if(name == "acc_rate")
  {
    target = &acc_rate;
    nvs_key = "acc_rate";
    default_value = DEFAULT_ACC_RATE;
    minimum_value = MIN_ACC_RATE;
    maximum_value = MAX_ACC_RATE;
    return true;
  }
  if(name == "spin_ramp_start_rpm")
  {
    target = &spin_ramp_start_rpm;
    nvs_key = "ramp_start_rpm";
    default_value = DEFAULT_SPIN_RAMP_START_RPM;
    minimum_value = MIN_SPIN_RAMP_START_RPM;
    maximum_value = MAX_SPIN_RAMP_START_RPM;
    return true;
  }
  if(name == "motor_full_scale_percent")
  {
    target = &motor_full_scale_percent;
    nvs_key = "motor_full_pct";
    default_value = DEFAULT_MOTOR_FULL_SCALE_PERCENT;
    minimum_value = MIN_MOTOR_FULL_SCALE_PERCENT;
    maximum_value = MAX_MOTOR_FULL_SCALE_PERCENT;
    return true;
  }
  if(name == "top_led_phase_offset_deg")
  {
    target = &top_led_phase_offset_deg;
    nvs_key = "top_led_trim";
    default_value = DEFAULT_TOP_LED_PHASE_OFFSET_DEG;
    minimum_value = MIN_LED_PHASE_OFFSET_DEG;
    maximum_value = MAX_LED_PHASE_OFFSET_DEG;
    return true;
  }
  if(name == "bottom_led_phase_offset_deg")
  {
    target = &bottom_led_phase_offset_deg;
    nvs_key = "bot_led_trim";
    default_value = DEFAULT_BOTTOM_LED_PHASE_OFFSET_DEG;
    minimum_value = MIN_LED_PHASE_OFFSET_DEG;
    maximum_value = MAX_LED_PHASE_OFFSET_DEG;
    return true;
  }
  return false;
}

bool drive_config_parse_float(const String &text, float &value)
{
  const char *start = text.c_str();
  char *end = nullptr;
  value = strtof(start, &end);
  if(end == start || !isfinite(value)) return false;
  while(*end != '\0' && isspace((unsigned char)*end)) end++;
  return *end == '\0';
}

bool drive_config_request_safe()
{
  return wifi_mode_active && robot_state == ROBOT_WIFI &&
         motor_output_gate_open() == false && motorL == 0 && motorR == 0;
}

void session_send_config_json()
{
  char response[1600];
  snprintf(response, sizeof(response),
      "{\"translation_scale_factor\":%.3f,\"percentdecel\":%.3f,\"acc_rate\":%.6f,"
      "\"spin_ramp_start_rpm\":%.6f,\"motor_full_scale_percent\":%.3f,"
      "\"top_led_phase_offset_deg\":%.3f,\"bottom_led_phase_offset_deg\":%.3f,"
      "\"defaults\":{\"translation_scale_factor\":%.3f,\"percentdecel\":%.3f,\"acc_rate\":%.6f,"
      "\"spin_ramp_start_rpm\":%.6f,\"motor_full_scale_percent\":%.3f,"
      "\"top_led_phase_offset_deg\":%.3f,\"bottom_led_phase_offset_deg\":%.3f},"
      "\"limits\":{\"translation_scale_factor\":[%.3f,%.3f],\"percentdecel\":[%.3f,%.3f],"
      "\"acc_rate\":[%.3f,%.3f],\"spin_ramp_start_rpm\":[%.3f,%.3f],"
      "\"motor_full_scale_percent\":[%.3f,%.3f],"
      "\"top_led_phase_offset_deg\":[%.3f,%.3f],"
      "\"bottom_led_phase_offset_deg\":[%.3f,%.3f]},"
      "\"spin_curve\":{\"floor_command\":%.3f,\"max_command\":999,\"exponent\":%.6f}}",
      translation_scale_factor, percentdecel, acc_rate, spin_ramp_start_rpm,
      motor_full_scale_percent,
      top_led_phase_offset_deg, bottom_led_phase_offset_deg,
      DEFAULT_TRANSLATION_SCALE_FACTOR, DEFAULT_PERCENT_DECEL, DEFAULT_ACC_RATE,
      DEFAULT_SPIN_RAMP_START_RPM, DEFAULT_MOTOR_FULL_SCALE_PERCENT,
      DEFAULT_TOP_LED_PHASE_OFFSET_DEG, DEFAULT_BOTTOM_LED_PHASE_OFFSET_DEG,
      MIN_TRANSLATION_SCALE_FACTOR, MAX_TRANSLATION_SCALE_FACTOR,
      MIN_PERCENT_DECEL, MAX_PERCENT_DECEL, MIN_ACC_RATE, MAX_ACC_RATE,
      MIN_SPIN_RAMP_START_RPM, MAX_SPIN_RAMP_START_RPM,
      MIN_MOTOR_FULL_SCALE_PERCENT, MAX_MOTOR_FULL_SCALE_PERCENT,
      MIN_LED_PHASE_OFFSET_DEG, MAX_LED_PHASE_OFFSET_DEG,
      MIN_LED_PHASE_OFFSET_DEG, MAX_LED_PHASE_OFFSET_DEG,
      SPIN_ACCEL_MIN_COMMAND, SPIN_ACCEL_EXPONENT);
  diagnostic_server.sendHeader("Cache-Control", "no-store");
  diagnostic_server.send(200, "application/json", response);
}

void session_send_config_error(int status, const char *message)
{
  char response[220];
  snprintf(response, sizeof(response), "{\"ok\":false,\"error\":\"%s\"}", message);
  diagnostic_server.send(status, "application/json", response);
}

void session_handle_config_update(bool reset_to_default)
{
  if(!drive_config_request_safe())
  {
    session_send_config_error(409, "Configuration is allowed only in safe WiFi mode");
    return;
  }
  if(!diagnostic_server.hasArg("name"))
  {
    session_send_config_error(400, "Missing setting name");
    return;
  }

  String name = diagnostic_server.arg("name");
  float *target = nullptr;
  const char *nvs_key = nullptr;
  float default_value = 0.0f;
  float minimum_value = 0.0f;
  float maximum_value = 0.0f;
  if(!drive_config_metadata(name, target, nvs_key, default_value,
                            minimum_value, maximum_value))
  {
    session_send_config_error(404, "Unknown setting");
    return;
  }

  float requested_value = default_value;
  if(!reset_to_default)
  {
    if(!diagnostic_server.hasArg("value") ||
       !drive_config_parse_float(diagnostic_server.arg("value"), requested_value))
    {
      session_send_config_error(400, "Value must be a finite number");
      return;
    }
  }
  if(requested_value < minimum_value || requested_value > maximum_value)
  {
    session_send_config_error(422, "Value is outside the allowed range");
    return;
  }

  if(config_store.putFloat(nvs_key, requested_value) != sizeof(float))
  {
    session_send_config_error(500, "Controller storage write failed");
    return;
  }
  float stored_value = config_store.getFloat(nvs_key, NAN);
  if(!isfinite(stored_value) || fabsf(stored_value - requested_value) > 0.0005f)
  {
    session_send_config_error(500, "Controller storage verification failed");
    return;
  }
  *target = stored_value;
  DEBUG_LOGF("web config: %s=%0.3f%s", name.c_str(), requested_value,
             reset_to_default ? " (default reset)" : "");

  char response[240];
  snprintf(response, sizeof(response),
           "{\"ok\":true,\"name\":\"%s\",\"value\":%.6f,\"default\":%.6f}",
           name.c_str(), stored_value, default_value);
  diagnostic_server.sendHeader("Cache-Control", "no-store");
  diagnostic_server.send(200, "application/json", response);
}

void session_web_register_routes()
{
  if(session_web_routes_registered) return;
  diagnostic_server.on("/", HTTP_GET, []() {
    diagnostic_server.sendHeader("Cache-Control", "no-store");
    diagnostic_server.send_P(200, "text/html", SESSION_DASHBOARD_HTML);
  });
  diagnostic_server.on("/api/live.json", HTTP_GET, session_handle_live_json);
  diagnostic_server.on("/api/summary.json", HTTP_GET, session_handle_summary_json);
  diagnostic_server.on("/api/timeline.csv", HTTP_GET, []() { session_handle_timeline_csv(false); });
  diagnostic_server.on("/api/melty.csv", HTTP_GET, []() { session_handle_timeline_csv(true); });
  diagnostic_server.on("/api/events.txt", HTTP_GET, session_handle_events_text);
  diagnostic_server.on("/api/timeline.json", HTTP_GET, session_handle_timeline_json);
  diagnostic_server.on("/api/config.json", HTTP_GET, session_send_config_json);
  diagnostic_server.on("/api/config", HTTP_POST,
                       []() { session_handle_config_update(false); });
  diagnostic_server.on("/api/config/reset", HTTP_POST,
                       []() { session_handle_config_update(true); });
  diagnostic_server.onNotFound([]() {
    if(diagnostic_server.uri().startsWith("/api/"))
    {
      session_send_config_error(404, "API route not found");
      return;
    }
    if(diagnostic_server.uri() == "/favicon.ico")
    {
      diagnostic_server.send(204, "text/plain", "");
      return;
    }
    // Captive-portal checks use several vendor-specific paths and hostnames.
    // Wildcard DNS brings them here; redirect them to the stable AP address.
    diagnostic_server.sendHeader("Cache-Control", "no-store");
    diagnostic_server.sendHeader("Location", "http://192.168.4.1/", true);
    diagnostic_server.send(302, "text/plain", "Open Vibemelt diagnostics");
  });
  session_web_routes_registered = true;
}

void session_web_start()
{
  session_web_register_routes();
  diagnostic_server.begin();
}

void session_web_stop()
{
  diagnostic_server.stop();
}

void session_web_service()
{
  diagnostic_server.handleClient();
}
