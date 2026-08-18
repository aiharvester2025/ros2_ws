// LiDAR inset: Canvas top-down (x-y) scatter, range-coloured, <=2000 pts.
import QtQuick 2.12

Rectangle {
    id: inset
    color: "#0b0f14"
    radius: 6
    border.color: "#22303f"
    border.width: 1

    property real range_limit_m: 8.0   // half-width of the plotted window

    Text {
        id: title
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.margins: 6
        text: "lidar top-down (x-y)  ±" + inset.range_limit_m + " m"
        color: "#9fb4c7"
        font.pixelSize: 11
    }

    Canvas {
        id: canvas
        anchors.top: title.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 6
        antialiasing: false

        onPaint: {
            var ctx = getContext("2d");
            ctx.reset();
            ctx.fillStyle = "#0b0f14";
            ctx.fillRect(0, 0, width, height);

            var centre_x = width / 2;
            var centre_y = height / 2;
            var scale = Math.min(width, height) / 2 / inset.range_limit_m;

            // Range rings + axes.
            ctx.strokeStyle = "#1c2833";
            for (var ring = 2; ring <= inset.range_limit_m; ring += 2) {
                ctx.beginPath();
                ctx.arc(centre_x, centre_y, ring * scale, 0, Math.PI * 2);
                ctx.stroke();
            }
            ctx.beginPath();
            ctx.moveTo(centre_x, 0); ctx.lineTo(centre_x, height);
            ctx.moveTo(0, centre_y); ctx.lineTo(width, centre_y);
            ctx.stroke();

            var points = bridge.lidarPoints;
            for (var i = 0; i < points.length; i++) {
                var p = points[i];
                var x = p[0], y = p[1], z = p[2];
                var px = centre_x + x * scale;
                var py = centre_y - y * scale;
                if (px < 0 || px >= width || py < 0 || py >= height) continue;
                var distance = Math.sqrt(x * x + y * y + z * z);
                var t = Math.min(distance / inset.range_limit_m, 1.0);
                // Cool (near) to warm (far) ramp matching the depth view.
                var r = Math.round(40 + t * 215);
                var g = Math.round(180 * (1 - Math.abs(t - 0.5) * 2));
                var b = Math.round(255 * (1 - t));
                ctx.fillStyle = "rgb(" + r + "," + g + "," + b + ")";
                ctx.fillRect(px - 1, py - 1, 2.4, 2.4);
            }

            // Vehicle marker at the origin.
            ctx.fillStyle = "#4fc3f7";
            ctx.fillRect(centre_x - 3, centre_y - 3, 6, 6);
        }
    }

    Connections {
        target: bridge
        onLidar_points_changed: canvas.requestPaint()
    }
}
