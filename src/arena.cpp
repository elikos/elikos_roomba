#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

static const double LOOP_RATE = 1.0;
static const std::string NODE_NAME = "arena";
static const std::string TF_MIDDLE_ARENA = "/elikos_arena_origin";
static const std::string ARENA_SQUARES_TOPIC_NAME = "arena_markers";
static const std::string ARENA_SQUARE_MODEL = "package://elikos_roomba/models/arena_square.dae";
static const double SQUARE_MODEL_OFFSET_X = 0.5;        // Offset to get the "origin" on the (0,0) corner 
static const double SQUARE_MODEL_OFFSET_Y = 0.5;        // Offset to get the "origin" on the (0,0) corner 
static const double LINE_WIDTH = 0.1;

visualization_msgs::MarkerArray markerarray_msg;
ros::Publisher arenasquares_pub;

// Create marker msg for green and red lines
std::vector<visualization_msgs::Marker> createLineMarkers(int dim) {
    visualization_msgs::Marker green_line, red_line;

    // green line
    green_line.header.frame_id = TF_MIDDLE_ARENA;
    green_line.id = dim*dim + 1;
    green_line.type = visualization_msgs::Marker::LINE_STRIP;
    green_line.action = visualization_msgs::Marker::ADD;
    green_line.mesh_resource = ARENA_SQUARE_MODEL;
    green_line.pose.orientation.w = 1.0;
    green_line.scale.x = LINE_WIDTH;
    green_line.lifetime = ros::Duration();
    green_line.color.g = 1.0;
    green_line.color.a = 1.0;
    geometry_msgs::Point pg1, pg2;
    pg1.x = - (double)dim / 2;
    pg1.y = (double)dim / 2;
    pg2.x = (double)dim / 2;
    pg2.y = (double)dim / 2;
    green_line.points.push_back(pg1);
    green_line.points.push_back(pg2);

    // red line
    red_line.header.frame_id = TF_MIDDLE_ARENA;
    red_line.id = dim*dim + 2;
    red_line.type = visualization_msgs::Marker::LINE_STRIP;
    red_line.action = visualization_msgs::Marker::ADD;
    red_line.mesh_resource = ARENA_SQUARE_MODEL;
    red_line.pose.orientation.w = 1.0;
    red_line.scale.x = LINE_WIDTH;
    red_line.lifetime = ros::Duration();
    red_line.color.r = 1.0;
    red_line.color.a = 1.0;
    geometry_msgs::Point pr1, pr2;
    pr1.x = - (double)dim / 2;
    pr1.y = - (double)dim / 2;
    pr2.x = (double)dim / 2;
    pr2.y = - (double)dim / 2;
    red_line.points.push_back(pr1);
    red_line.points.push_back(pr2);

    std::vector<visualization_msgs::Marker> lines;
    lines.push_back(green_line);
    lines.push_back(red_line);
    return lines;
}

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

    // squares
    for (int i = 0; i < dim; ++i) {
        double coord_x = i;
        for (int j = 0; j < dim; ++j) {
            double coord_y = j;
            markers.push_back(createMarkerMsg(coord_x - ((double)dim / 2), 
                                                coord_y - ((double)dim / 2),
                                                (i * dim) + j));
        }
    }
    // lines
    std::vector<visualization_msgs::Marker> lines = createLineMarkers(dim);
    markers.insert(markers.end(), lines.begin(), lines.end());

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