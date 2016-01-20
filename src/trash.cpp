
void test()
{
	float f = 0.6;
	unsigned char *pc;
	pc = (unsigned char*)&f;
	*(unsigned int*)&f = (pc[3] << 24) | (pc[2] << 16) | (pc[1] << 8) | (pc[0] << 0);
	ROS_INFO("%d",pc[0]);
	ROS_INFO("%d",pc[1]);
	ROS_INFO("%d",pc[2]);
	ROS_INFO("%d",pc[3]);
	ROS_INFO("--->%f<---",f);
}

