import utm
import rclpy
import rclpy.logging
from rclpy.node import Node

from privyaznik_msgs.srv import UtmToWgs, WgsToUtm


class CoordinateTransferService(Node):

    def __init__(self):
        super().__init__('coordinate_transfer_service')
        self.utm_to_wgs_srv = self.create_service(UtmToWgs, 'utm_to_wgs', self.utm_to_wgs_cb)
        self.wgs_to_utm_srv = self.create_service(WgsToUtm, 'wgs_to_utm', self.wgs_to_utm_cb)

    def utm_to_wgs_cb(self, request: UtmToWgs.Request, response: UtmToWgs.Response):
        response.latitude, response.longitude = utm.to_latlon(easting=request.easting, northing=request.northing, zone_number=request.zone_number, zone_letter=request.zone_letter, strict=True)

        return response

    def wgs_to_utm_cb(self, request: WgsToUtm.Request, response: WgsToUtm.Response):
        # response.latitude, response.longitude = utm.to_latlon(request.easting, request.northing, request.zone_number, request.zone_letter, request.northern, True)
        response.easting, response.northing, response.zone_number, response.zone_letter = utm.from_latlon(request.latitude, request.longitude)
        self.get_logger().info('AAA')
        print(response)
        return response


def main(args=None):
    rclpy.init(args=args)

    node = CoordinateTransferService()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
