import {writeFileSync} from 'node:fs';
import {randomUUID} from 'node:crypto';
const ws=new WebSocket('ws://127.0.0.1:8765','foxglove.sdk.v1');ws.binaryType='arraybuffer';
const id='ws-static-'+randomUUID(),answers=[];let sent=false,closed=false,failure='',answerChannel,payload,lastPublish=0,publishCount=0;
function publishQuestion(){
 if(sent||answerChannel===undefined)return;sent=true;
 ws.send(JSON.stringify({op:'subscribe',subscriptions:[{id:7,channelId:answerChannel}]}));
 ws.send(JSON.stringify({op:'advertise',channels:[{id:1,topic:'/omnifleet_t2/diagnostics/question',encoding:'cdr',schemaName:'std_msgs/msg/String',schemaEncoding:'ros2msg',schema:'string data\n'}]}));
 const text=Buffer.from(JSON.stringify({request_id:id,question:'仅做静态核对：104和113是否都在使用113的公共导航底图？请分别给出底图来源、全局坐标、分辨率和导航就绪结论，指出尚未验证的项。保持当前禁止运动的状态。'}));
 payload=Buffer.alloc(13+text.length+1);payload[0]=1;payload.writeUInt32LE(1,1);payload[6]=1;
 payload.writeUInt32LE(text.length+1,9);text.copy(payload,13);lastPublish=Date.now();
}
ws.addEventListener('message',event=>{
 if(typeof event.data==='string'){
  const m=JSON.parse(event.data);
  if(m.op==='advertise'){
   const channel=m.channels.find(c=>c.topic==='/omnifleet_t2/diagnostics/answer');
   if(channel&&sent&&channel.id!==answerChannel){ws.send(JSON.stringify({op:'unsubscribe',subscriptionIds:[7]}));ws.send(JSON.stringify({op:'subscribe',subscriptions:[{id:7,channelId:channel.id}]}));}
   if(channel)answerChannel=channel.id;
   publishQuestion();
  }
  if(m.op==='status'&&m.level==='error')failure=m.message;
  return;
 }
 const data=Buffer.from(event.data);
 if(data[0]!==1||data.length<21||data.readUInt32LE(1)!==7)return;
 const length=data.readUInt32LE(17);
 try{const m=JSON.parse(data.subarray(21,21+length-1).toString());if(m.request_id===id)answers.push(m);}catch{}
});
ws.addEventListener('error',event=>{if(!closed)failure=event.message||'websocket error';});
const started=Date.now();
while(Date.now()-started<130000&&!failure&&!answers.some(a=>['complete','error'].includes(a.status))){
 await new Promise(r=>setTimeout(r,200));
 if(payload&&!answers.length&&publishCount<4&&Date.now()-lastPublish>(publishCount?3000:1000)){ws.send(payload);lastPublish=Date.now();publishCount++;}
 if(!answers.length&&Date.now()-started>18000){failure='request acknowledgement not received';break;}
}
closed=true;ws.close();
const last=answers.at(-1),result={request_id:id,transport:'foxglove.sdk.v1 / CDR',seconds:(Date.now()-started)/1000,sent,publishCount,failure,answer:last,stages:answers.map(a=>a.stage||a.status)};
writeFileSync('/home/iecme/robot_backups/msc_shared_map_dsh_20260909/websocket-diagnostic.json',JSON.stringify(result,null,2));
console.log(JSON.stringify(result));
if(!last||last.status!=='complete'||!['发现的问题','判断依据','处理步骤'].every(s=>last.answer.includes(s)))process.exitCode=1;
