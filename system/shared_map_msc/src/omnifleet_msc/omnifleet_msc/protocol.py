import hashlib,hmac,http.client,json,time,uuid
from urllib.parse import urlsplit

def signature(key,context,nonce,body):
    return hmac.new(key.encode(),context.encode()+b'\n'+nonce.encode()+b'\n'+body,hashlib.sha256).hexdigest()

def request(url,identity,key,path,data,timeout=.7):
    address=urlsplit(url)
    if address.scheme!='http':raise ValueError('only configured LAN HTTP endpoint supported')
    body=json.dumps(data,separators=(',',':'),allow_nan=False).encode()
    nonce=str(time.time())+':'+uuid.uuid4().hex
    conn=http.client.HTTPConnection(address.hostname,address.port or 80,timeout=timeout)
    try:
        conn.request('POST',path,body,{'Content-Type':'application/json','X-MSC-ID':identity,
            'X-MSC-Nonce':nonce,'X-MSC-Signature':signature(key,'POST '+path,nonce,body)})
        response=conn.getresponse();payload=response.read(2_000_001)
        if len(payload)>2_000_000:raise ValueError('response too large')
        expected=signature(key,'RESPONSE '+path,nonce,payload)
        if not hmac.compare_digest(response.getheader('X-MSC-Signature',''),expected):raise ValueError('invalid response signature')
        result=json.loads(payload)
        if response.status!=200:raise ValueError(result.get('error','coordinator rejected request'))
        return result
    finally:conn.close()
