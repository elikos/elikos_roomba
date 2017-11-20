#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

static const double LOOP_RATE = 1.0;
static const std::string NODE_NAME = "arena";
static const std::string TF_MIDDLE_ARENA = "/elikos_arena_origin";
static const std::string ARENA_SQUARES_TOPIC_NAME = "/arena_squares";
static const std::string ARENA_SQUARE_MODEL = "package://elikos_roomba/models/arena_square.dae";
static const double SQUARE_MODEL_OFFSET_X = 0.5;        // Offset to get the "origin" on the (0,0) corner 
static const double SQUARE_MODEL_OFFSET_Y = 0.5;        // Offset to get the "origin" on the (0,0) corner 

visualization_msgs::MarkerArray markerarray_msg;
ros::Publisher arenasquares_pub;

// Create marker msg from (x,y) coords and ID 
visualization_msgs::Marker createMarkerMsg(double x, double y, int id) {
    visualization_msgs::Marker msg;
    msg.header.frame_id = TF_MIDDLE_ARENA;
    msg.id = id;
    msg.type = visualization_msgs::Marker::MESH_RESOURCE;
    msg.action = visualization_msgs::Marker::ADD;
    msg.mesh_resource = ARENA_SQUARE_MODEL;
    msg.pose.position.x = x + SQUARE_MODEL_OFFSET_X;
    msg.pose.position.y = y + SQUARE_MODEL_OFFSET_Y;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = 1.0;
    msg.scale.y = 1.0;
    msg.scale.z = 1.0;
    msg.mesh_use_embedded_materials = true;
    msg.lifetime = ros::Duration();
    return msg;
}

// Create one-time marker array
void createMarkerArray(int dim) {
    std::vector<visualization_msgs::Marker> markers;
    for (int i = 0; i < dim; ++i) {
        double coord_x = i;
        for (int j = 0; j < dim; ++j) {
            double coord_y = j;
            markers.push_back(createMarkerMsg(coord_x - ((double)dim / 2), 
                                                coord_y - ((double)dim / 2),
                                                (i * dim) + j));
        }
    }
    markerarray_msg.markers = markers;
}

// Publish marker array after updating timestamps 
void publishMarkerArray() {
    for (int i = 0; i < markerarray_msg.markers.size(); ++i) {
        markerarray_msg.markers[i].header.stamp = ros::Time::now();
    }
    arenasquares_pub.publish(markerarray_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    ros::NodeHandle n_p("~");
    int arena_dimension;
    n_p.getParam("arena_dimension", arena_dimension);

    arenasquares_pub = n.advertise<visualization_msgs::MarkerArray>(ARENA_SQUARES_TOPIC_NAME, 1);
    
    createMarkerArray(arena_dimension);

    ros::Rate rate(LOOP_RATE);
    while (ros::ok())
    {
        publishMarkerArray();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}