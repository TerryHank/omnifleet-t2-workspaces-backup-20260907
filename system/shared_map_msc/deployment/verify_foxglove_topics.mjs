import {writeFileSync} from 'node:fs';
const topics=new Set(),ws=new WebSocket('ws://127.0.0.1:8765','foxglove.sdk.v1');
const operations=[];let opened=false,error='',closing=false;
ws.addEventListener('open',()=>{opened=true;});
ws.addEventListener('message',event=>{
  if(typeof event.data!=='string')return;
  const data=JSON.parse(event.data);
  operations.push(data.op);
  if(data.op==='advertise')for(const channel of data.channels)topics.add(channel.topic);
});
ws.addEventListener('error',event=>{if(!closing){error=event.message;process.exitCode=1;}});
await new Promise(resolve=>setTimeout(resolve,5000));closing=true;ws.close();
const required=['/fleet/map','/fleet/status','/fleet/report','/fleet/robots','/omnifleet_t2/diagnostics/answer'];
const result={opened,error,operations,channel_count:topics.size,required,advertised:required.filter(t=>topics.has(t)),missing:required.filter(t=>!topics.has(t))};
writeFileSync('/home/iecme/robot_backups/msc_shared_map_dsh_20260909/foxglove-topics.json',JSON.stringify(result,null,2));
console.log(JSON.stringify(result));if(result.missing.length)process.exitCode=1;
