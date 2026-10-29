module drone.dronepathplanning_bfs {
    requires javafx.controls;
    requires javafx.fxml;


    opens drone.dronepathplanning_bfs to javafx.fxml;
    exports drone.dronepathplanning_bfs;
}