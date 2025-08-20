import time
import json
from flask import Flask, request, render_template, jsonify,  Response

from gnss_read import GNSSData 
from generate_path import Path_plan
from run import Run_process
from utils.geodesy import Geodesy
app = Flask(__name__)
gcp=[]

#data_read = GNSSData("/home/robo/dev/fil_/path_deployment/src/canbus/serial_posix_v2/inc/gnr.buf")
proc = Run_process("../src/","gnss")


data_read = GNSSData("../data/gnss.json")


def event_stream():
    data = data_read.last_data()
    with open("../data/live_log.txt", "a") as logfile:  
        while True:
            try:
                data = data_read.last_data()
                if data:
                    logfile.write(json.dumps(data) + "\n")  
                    logfile.flush()  
                    yield f"data: {json.dumps(data)}\n\n"
                else:
                    yield "data: {}\n\n"
                #time.sleep(0.5)   # optional throttle
            except Exception as e:
                print(f"Error in stream: {e}")
                break

                
@app.route("/")
def root():
    return render_template("page1.html")

@app.route('/foo', methods=['POST']) 
def foo():

    gcp.clear()
    data = request.json 
    f = open('../data/data_log.txt', 'w')
    for item in data:
       f.write("%s\n" % item)
    f.close()

    outt=[]
    #print(data[0])
    for n in range(0,len(data[0])):
        outt=[[data[len(data)-1][n-1]["lat"],data[len(data)-1][n-1]["lng"]],[data[len(data)-1][n]["lat"],data[len(data)-1][n]["lng"]]]
        gcp.append(outt)

    return jsonify(data)



@app.route('/livelocation', methods=['GET'])
def livelocation():
    #proc.runobject()
    #time.sleep(1)
    open("../data/live_log.txt", "w").close() 

    return Response(event_stream(), mimetype="text/event-stream")

@app.route('/stop', methods=['GET'])
def stop():
    response = proc.stopobject()
    res = {"response":response}
    res = {"response":"stopped"}
    return jsonify(res)
    
@app.route('/simplify')
def simplify():
    global gcp
    gcp.clear()
    
    # Read JSON objects from file
    with open("../data/live_log.txt", "r") as f:
            data = [json.loads(line) for line in f if line.strip()]

    # Extract coords (lon, lat)
    coords = [(obj["longitude"], obj["latitude"]) for obj in data]
    out=Geodesy.simplify_from_file(coords)
    #get data from json into a list
    #out=[[28.43125819745395, 77.33779281377794],[28.431364339125686,77.33813881874086],[28.431163849211924, 77.33842045068742],[28.430958641495092,77.33802884817125]]
    #print(len(out))
    
    outt=[]

    for n in range(0,len(out)):
        outt=[[out[n-1][0],out[n-1][1]],[out[n][0],out[n][1]]]
        gcp.append(outt)
    
    info={}
    info["boundary"]=gcp

    return jsonify(info)

@app.route('/path', methods=['GET']) 
def path():
    infor={}
    print("gcp length",len(gcp))
    out= Path_plan(gcp,2.2,3.4,2.2)
    tp,headland=out.path()
    #print("tp: ",tp)

    infor["track"]=tp
    infor["headland"]=headland

    return jsonify(infor)         

if __name__ == '__main__':
    try:
        app.run(host="0.0.0.0", port=3000, threaded=True, debug=True)

    except KeyboardInterrupt:
        response = proc.stopobject()
    except Exception as e:
        response = proc.stopobject()
