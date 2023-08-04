# people_msgs_utils

This package provides the necessary utilities to convert between a standard [`people` detection stack](https://wiki.ros.org/people) and [`spencer_people_tracking`](https://github.com/spencer-project/spencer_people_tracking) stack.

To make use of multiple data provided by `spencer_people_tracking`, [`tagnames` and `tags` fields](https://github.com/wg-perception/people/blob/melodic/people_msgs/msg/Person.msg#L5) are used to carry additional data. String vectors in `people_msgs` message structure allow to freely extend items carried by the messages. However, this requires parsing those strings appropriately. Making use of `tags` field also prevents legacy packages from breaking.

A library included in this package allows convenient parsing of the information from `spencer_people_tracking` coded in `tagnames` and `tags` of standard `people_msgs` according to [this](https://github.com/rayvburn/spencer_people_tracking/pull/4) PR.

Once the original scheme of filling the `tagnames` and `tags` fields ([reference](https://github.com/rayvburn/spencer_people_tracking/blob/41b0e6362310b40a3ebf8c84eebe052e3f321855/tracking/people/spencer_tracking_conversion/scripts/conversion_utils.py#L59C1-L59C1)) is preserved, classes introduced in this package can also be used to decode data obtained with other human perception stacks.
