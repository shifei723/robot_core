#!/usr/bin/python3
"""YOLOX-tiny COCO semantic proposals; motion is decided by depth tracking, never by class."""
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np

COCO = ('person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant stop_sign parking_meter bench bird cat dog horse sheep cow elephant bear zebra giraffe backpack umbrella handbag tie suitcase frisbee skis snowboard sports_ball kite baseball_bat baseball_glove skateboard surfboard tennis_racket bottle wine_glass cup fork knife spoon bowl banana apple sandwich orange broccoli carrot hot_dog pizza donut cake chair couch potted_plant bed dining_table toilet tv laptop mouse remote keyboard cell_phone microwave oven toaster sink refrigerator book clock vase scissors teddy_bear hair_drier toothbrush').split()


class YOLOSemantics:
    def __init__(self, model, confidence=.45):
        self.net=cv2.dnn.readNetFromONNX(str(model))
        self.confidence=confidence
        grids=[];strides=[]
        for stride in (8,16,32):
            x,y=np.meshgrid(np.arange(416//stride),np.arange(416//stride))
            grids.append(np.stack((x,y),axis=2).reshape(-1,2));strides.append(np.full((len(grids[-1]),1),stride))
        self.grid=np.concatenate(grids);self.stride=np.concatenate(strides)

    def infer(self, image):
        h,w=image.shape[:2];ratio=min(416/h,416/w)
        padded=np.full((416,416,3),114,np.uint8)
        resized=cv2.resize(image,(int(w*ratio),int(h*ratio)))
        padded[:resized.shape[0],:resized.shape[1]]=resized
        # Official YOLOX deployment preprocessing: BGR, 0..255, no mean/std.
        self.net.setInput(cv2.dnn.blobFromImage(padded))
        pred=self.net.forward()[0]
        centers=(pred[:,:2]+self.grid)*self.stride
        sizes=np.exp(np.clip(pred[:,2:4],-20,20))*self.stride
        scores=pred[:,4,None]*pred[:,5:]
        classes=np.argmax(scores,axis=1);conf=scores[np.arange(len(scores)),classes]
        boxes=np.column_stack((centers-sizes/2,sizes))/ratio # x,y,w,h
        result=[]
        for cls in np.unique(classes[conf>=self.confidence]):
            indices=np.flatnonzero((classes==cls)&(conf>=self.confidence))
            keep=cv2.dnn.NMSBoxes(boxes[indices].tolist(),conf[indices].tolist(),self.confidence,.45)
            for i in np.asarray(keep,dtype=int).ravel():
                index=indices[i];x,y,bw,bh=boxes[index]
                x1,y1=max(0.,float(x)),max(0.,float(y));x2,y2=min(float(w),float(x+bw)),min(float(h),float(y+bh))
                if x2>x1 and y2>y1:result.append((x1,y1,x2,y2,int(cls),float(conf[index])))
        return result


def main():
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray,Detection2D,ObjectHypothesisWithPose
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',default=str(Path(__file__).resolve().parents[1]/'models/yolox_tiny.onnx'))
    parser.add_argument('--calibration',required=True)
    parser.add_argument('--hz',type=float,default=5.)
    args=parser.parse_args()
    cv2.setNumThreads(2)
    detector=YOLOSemantics(args.model)
    calibration=json.loads(Path(args.calibration).read_text())
    fx,fy,cx,cy=calibration['color_intrinsics'];k=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],float)
    maps=cv2.initUndistortRectifyMap(k,np.array(calibration['color_distortion']),None,k,tuple(calibration['color_size']),cv2.CV_32FC1)
    rclpy.init();node=rclpy.create_node('yolo_semantic_assist');bridge=CvBridge();latest=[None];last=[None]
    pub=node.create_publisher(Detection2DArray,'/detection/semantic_boxes',5)
    overlay=node.create_publisher(Image,'/detection/semantic_image',2)
    status=node.create_publisher(String,'/detection/semantic_status',5)
    def image_cb(msg):latest[0]=msg
    subscription=node.create_subscription(Image,'/camera/color/image_raw',image_cb,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
    def process():
        msg=latest[0]
        if msg is None:return
        stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if stamp==last[0]:return
        age=node.get_clock().now().nanoseconds*1e-9-(stamp[0]+stamp[1]*1e-9)
        if age<-.05 or age>.15:return
        last[0]=stamp;start=time.monotonic()
        image=cv2.remap(bridge.imgmsg_to_cv2(msg,'bgr8'),*maps,cv2.INTER_LINEAR)
        detections=detector.infer(image)
        output=Detection2DArray();output.header=msg.header
        for x1,y1,x2,y2,cls,score in detections:
            detection=Detection2D();detection.header=msg.header
            detection.bbox.center.position.x=(x1+x2)/2;detection.bbox.center.position.y=(y1+y2)/2
            detection.bbox.size_x=x2-x1;detection.bbox.size_y=y2-y1
            hypothesis=ObjectHypothesisWithPose();hypothesis.hypothesis.class_id=COCO[cls];hypothesis.hypothesis.score=score
            detection.results=[hypothesis];output.detections.append(detection)
            cv2.rectangle(image,(int(x1),int(y1)),(int(x2),int(y2)),(0,200,255),2)
            cv2.putText(image,f'{COCO[cls]} {score:.2f}',(int(x1),max(15,int(y1)-5)),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,200,255),1)
        pub.publish(output)
        image_msg=bridge.cv2_to_imgmsg(image,'bgr8');image_msg.header=msg.header;overlay.publish(image_msg)
        diagnostic=String();diagnostic.data=json.dumps(dict(model='YOLOX-tiny',detections=len(detections),inference_ms=round((time.monotonic()-start)*1000,2),input_age_ms=round(age*1000,2)))
        status.publish(diagnostic)
    timer=node.create_timer(1/args.hz,process)
    node.get_logger().info('YOLO semantics enabled; object class alone never declares motion')
    rclpy.spin(node)


if __name__=='__main__':main()
