# people_msgs_utils

This package provides necessary utilities to convert between standard [`people` detection stack](https://wiki.ros.org/people) and [`spencer_people_tracking`](https://github.com/spencer-project/spencer_people_tracking) stack.

To make use of multiple data provided by `spencer_people_tracking`, [`tagnames` and `tags` fields](https://github.com/wg-perception/people/blob/melodic/people_msgs/msg/Person.msg#L5) are used to carry additional data. String vectors in message structures allow to freely extend items carries by the messages, however it also requires to parse those strings appropriately. Making use of `tags` field also prevents legacy packages from breaking.

Library included in this package allows to conveniently parse information from `spencer_people_tracking` coded in `tagnames` and `tags` of standard `people_msgs` according to [this](https://github.com/rayvburn/spencer_people_tracking/pull/4) PR.
