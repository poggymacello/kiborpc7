#!/usr/bin/env python
# Summarize nav topic timing in a rosbag: gaps, last message, positions.
import sys
import rosbag

bag = rosbag.Bag(sys.argv[1])
t_lo = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
topics = ["/sensor_fusion/navigation", "/nav/true/navigation", "/ctl/wrench", "/ctl/status", "/clock"]
info = bag.get_type_and_topic_info()[1]
for t in sorted(info):
    if "nav" in t or "ctl" in t or t == "/clock":
        print("topic %-45s n=%d" % (t, info[t].message_count))
last = {}
gaps = {}
count = {}
firstpos = {}
for topic, msg, bt in bag.read_messages(topics=topics):
    ts = bt.to_sec()
    if topic == "/clock":
        key = topic
        stamp = msg.clock.to_sec()
    else:
        key = topic
        try:
            stamp = msg.header.stamp.to_sec()
        except AttributeError:
            try:
                stamp = msg.pose.header.stamp.to_sec()
            except AttributeError:
                stamp = ts
    count[key] = count.get(key, 0) + 1
    if key in last:
        g = stamp - last[key][0]
        if g > 0.3:
            gaps.setdefault(key, []).append((last[key][0], stamp, g))
    last[key] = (stamp, ts)
    if "navigation" in topic:
        p = msg.pose.pose.position
        if key not in firstpos:
            firstpos[key] = (stamp, p.x, p.y, p.z)
        lastpos = (stamp, p.x, p.y, p.z, msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        last[key + "_pos"] = lastpos
for k in sorted(last):
    print("last %-45s %s" % (k, last[k]))
for k in sorted(firstpos):
    print("first %-45s %s" % (k, firstpos[k]))
for k in sorted(gaps):
    print("gaps %s: %d" % (k, len(gaps[k])))
    for g in gaps[k][:15]:
        print("   %.3f -> %.3f  (%.3f s)" % g)
