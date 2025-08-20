from math import atan2, radians, cos, sin, asin, sqrt , degrees, acos, pi, tan
import numpy as np
import math
from utils.generate_headland import GenerateHeadland
from utils.turn_generation import GenerateTurn
from utils.geodesy import Geodesy

class Path_plan:
    def __init__(self, gcp, application_width, turning_radius, tractor_wheelbase):
        """
        :param gcp- gcp pooints of path or boundary
        """
        self.gcp = gcp
        #Geodesy=Geodesy()
        self.application_width, self.turning_radius, self.tractor_wheelbase=application_width, turning_radius, tractor_wheelbase

        
    def path(self):

        tp,headland= self.path_planning(self.gcp,self.application_width, self.turning_radius, self.tractor_wheelbase)
        
        total_area = Geodesy.track_area(tp,self.application_width)
        #print(total_area)
        #self.save_path(tp)
        return tp,headland

    def save_path(self, data):
        f = open('../data/path_points.txt', 'w')
        for item in data:
            f.write(str(item[0])+","+str(item[1])+"\r\n")
        f.close()
        print("path data stored to file in data/path_points.txt")
     
    def rotate(self,track):
        
        xyz=[]
        if len(track)>1:
            for i in range(0,len(track)):
                xyz.append(track[(len(track)-1)-i])
        return xyz
                

    def track(self,gcp_1,gcp_2,scale_div):
        plot_pt = list()
        plot_pt.append(gcp_1)

        delta_L = gcp_2[1] - gcp_1[1]
        
        dist = Geodesy.distancebet(gcp_1,gcp_2)
        X = cos(radians(gcp_2[0])) * sin(radians(delta_L))
        Y = cos(radians(gcp_1[0])) * sin(radians(gcp_2[0])) - sin(radians(gcp_1[0])) * cos(radians(gcp_2[0])) * cos(radians(delta_L))
        beta = degrees(atan2(X,Y))

        if beta >= 0:
            final_bearing = beta
        else:
            final_bearing = 360 + beta

        earth_radius = 6378100
        data_div = scale_div
        number_pts = int(round(dist) / data_div)

        for i in range(0,number_pts):
            angular_dist = data_div/earth_radius
            new_lat = degrees(asin(sin(radians(gcp_1[0])) * cos(angular_dist) + cos(radians(gcp_1[0]))*sin(angular_dist)*cos(radians(beta))))
            new_long = gcp_1[1] + degrees(atan2(sin(radians(beta))*sin(angular_dist)*cos(radians(gcp_1[0])),cos(angular_dist)-sin(radians(gcp_1[0]))*sin(radians(new_lat))))
            plot_pt.append([new_lat,new_long]) 
            data_div = data_div + scale_div
        plot_pt.append(gcp_2)
        return plot_pt,dist,final_bearing

    def arange_tracks(self, lista, m):
        chunks = [lista[i:i+m] for i in range(0, len(lista), m)]
       
        if len(chunks) > 1 and len(chunks[-1]) < m:
            chunks[-2].extend(chunks[-1])
            chunks.pop() 
        
        return chunks

    def path_planning(self,gcp,Application_width,turning_radius,tractor_wheelbase):
    
        #define variables 
        final_track=[]
        filt_side=[]
        sides=[]
        sides_k=[]
        trak=[]
        green=[]
        inf_data={}
        blue=[]
        trakk=[]
        defa=[]
        #turnss=1 : uturn, turnss=2 : omega turn, turnss=3 : flat turn
        turnss=3
        
        
        
        
        polygon=[]
        for lo in range(0,len(gcp)):
            gcp_1 = gcp[lo][0]
            gcp_2 = gcp[lo][1]
            polygon.append(gcp_1)
            dist_data = Geodesy.distancebet(gcp_1,gcp_2)
            inf_data[((gcp_1[0],gcp_1[1]),(gcp_2[0],gcp_2[1]))]=(dist_data)
      
        
        area= Geodesy.area_of(polygon)
        #print(f"area of field = {area}")   
        coord = (max(inf_data, key=inf_data.get))
        lis=list(coord[0])
        coor=[]
        coor.append(lis)
        lis=list(coord[1])
        coor.append(lis)

        #coord_dist = Geodesy.distancebet(coord[0],coord[1])
        long_pts,long_dist,long_bearing = self.track(coord[0],coord[1],1)
        inf_data.clear()

        m=gcp.index(coor)
        
        gcpp=[]
        gcpp=(gcp[m:len(gcp)]).copy()
        gcpp.extend(gcp[:m+1])
        
        concave_edge=[]
        
        a = GenerateHeadland(long_bearing, Application_width, turning_radius)
        #identify the shape
        '''
        for i in range(0,len(gcpp)-1):
            
            angle_1 = Geodesy.norm_180(Geodesy.angle(gcpp[i][0],gcpp[i][1])[0])
            angle_2= Geodesy.norm_180(Geodesy.angle(gcpp[i+1][0],gcpp[i+1][1])[0])
            diff=Geodesy.norm_180(angle_1-angle_2)
            if diff>0:
                
                print(gcpp[i][1])
                concave_edge.append(gcpp[i][1])
                concave_edge.append(gcpp[i+1][0])'''
                

        
        

        '''for i in range(0,len(gcpp)-1):
            
            angle_1 = Geodesy.norm_180(Geodesy.angle(gcpp[i][0],gcpp[i][1])[0])
            angle_2= Geodesy.norm_180(Geodesy.angle(gcpp[i+1][0],gcpp[i+1][1])[0])
            diff=Geodesy.norm_180(angle_1-angle_2)
            if diff>0:
                #print("concave edge found")
                print(gcpp[i][1])
                concave_edge.append(gcpp[i][1])

            #print(f"angle between : {angle_1} and {angle_2} is {diff}")'''
        
        #print("concave data",concave_edge)
        '''for i in concave_edge:
            for j in gcpp:
                
                angle_1 = Geodesy.norm_180(Geodesy.angle(j[0],j[1])[0])
                for k in gcpp:
                    
                    angle_prev= Geodesy.norm_180(Geodesy.angle(i,k[0])[0])
                    angle_next= Geodesy.norm_180(Geodesy.angle(i,k[1])[0])
                    #print(f"given angle = {angle_1}, prev_angle ={angle_prev}, next angle ={angle_next}")
                    if (angle_prev<angle_1 and angle_1<angle_next):
                        print("pass")
                        print(f"given angle = {angle_1}, prev_angle ={angle_prev}, next angle ={angle_next}")
                        
                        a.determine_pt( k[0], angle_prev, i, angle_1)
                        
                print(":::::::::::::::::::::::::::::::::::::::::::::::::::::")'''
                
        

            
        
        
        #generate headland
        
        h_gcpp= a.gen_headland(gcpp)
        #h_gcpp=gcpp
           
  
        for lo in range(0,len(h_gcpp)):
            gcp_1 = h_gcpp[lo][0]
            gcp_2 = h_gcpp[lo][1]
            dist_data = Geodesy.distancebet(gcp_1,gcp_2)
            inf_data[((gcp_1[0],gcp_1[1]),(gcp_2[0],gcp_2[1]))]=(dist_data)
         
        coord = (max(inf_data, key=inf_data.get))    
        long_pts,long_dist,long_bearing = self.track(coord[0],coord[1],1)
        
        dis=0
        point=[]
        for n in inf_data:
            try:
                if n == coord:
                    continue

                theta, th=Geodesy.angle(n[0],n[1])
                delta=abs(long_bearing-theta)
                delta=min(delta,360-delta)
                
                diss=dis/sin(radians(delta))
                angular_dist=diss/6378100
            
                A_lat=degrees(asin(sin(radians(n[0][0])) * cos(angular_dist) + cos(radians(n[0][0]))*sin(angular_dist)*cos(radians(theta))))    
                A_long= n[0][1] + degrees(atan2(sin(radians(theta))*sin(angular_dist)*cos(radians(n[0][0])),cos(angular_dist)-sin(radians(n[0][0]))*sin(radians(A_lat))))
                point.append([A_lat,A_long])
                
                
                p=abs(Application_width/sin(radians(delta)))
                pok,nok,gok = self.track([point[0][0],point[0][1]],n[1],p)
                po,no,go = self.track(n[0],n[1],0.1)

                d=Geodesy.distancebet(pok[len(pok)-1],pok[len(pok)-2])
                
                
                if d<p:
                    pok.pop()
                    dis=Application_width-d*abs(sin(radians(delta)))

                green.append(pok)
                blue.append(po)
                point.clear()
            except:
                pass
        
        return green, h_gcpp
        for k in blue:
            for h in k:
                sides.append(h)
                   
        for k in green:
            for h in k:
                sides_k.append(h)    

        z_in=[]
        defa =[]         
        
        for i in range(0,len(sides_k)):
            z_in=[]
            for k in range(0,len(sides)):
                z_out = sides[k]
                sh,sh_dis = Geodesy.angle(z_out,sides_k[i])
                if int(sh) == int(long_bearing):
                    
                    dt = [z_out,sides_k[i]]
                       
                    z_in.append(dt)
                        
                    if len(z_in) > 1:
                        filt_side.append(z_in[0])
                    else:
                        filt_side.append(dt)
        defa =[]
        for x in filt_side:
            if x not in defa:
                defa.append(x)
        return defa, h_gcpp
        trakk=[]
        trakk=defa
        
        trakk=trakk[:]          
        result = self.arange_tracks(trakk, 9)
        list0=[]
        list1=[]
        count=0

        for i in result:
            k=int(len(i)/2)+1
                
            for j in range(0,k): 
                try:
                    trak.append(i[j])
                    try:
                        if count%2==0:    
                            list0.append(trakk.index(i[j]))
                            list0.append(trakk.index(i[j+k]))
                        else:
                            list1.append(trakk.index(i[j]))
                            list1.append(trakk.index(i[j+k]))
                    except:
                        pass
                    trak.append(i[j+k])
                except:
                    pass
            count+=1

        
        b=GenerateTurn()   
   
        for i in range(0,len(trak)):
            try:
                if i in list0:
                    if i%2==0:
                        final_track.append(trak[i])
                        final_track.append(self.rotate(b.flatturn(trak[i+1][len(trak[i])-1],trak[i][len(trak[i+1])-1],turning_radius)))
                    
                    if i%2==1:
                        final_track.append(self.rotate(trak[i]))
                        final_track.append(self.rotate(b.flatturn(trak[i+1][0],trak[i][0],turning_radius)))

                if i in list1:
                    if i%2==0:
                        final_track.append((trak[i]))
                        final_track.append((b.flatturn(trak[i][1],trak[i+1][1],turning_radius)))
                    if i%2==1:
                        final_track.append(self.rotate(trak[i]))
                        final_track.append((b.flatturn(trak[i][0],trak[i+1][0],turning_radius)))

            except:
                pass




        flat_track=[]
        final_track = [ele for ele in final_track if ele != []]
        count=0
        for i in range(0, len(final_track)):
            for j in range(0, len(final_track[i])):
                flat_track.append(final_track[i][j])
        
        
                               
        return flat_track,h_gcpp
            
