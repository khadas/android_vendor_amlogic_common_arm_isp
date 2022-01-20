/*
 *
 * Android control tool guide
 *
 */

1.code compile
	a.
		cp -r control_tool_android hardware/amlogic/camera

		cd control_tool_android/libevent

		mm
		
		there will output libisptool.a in android out dir,

		then copy libisptool.a to control_tool/act-server/lib

	b.	cd control_tool_android/act-server

		mm

		there will output act-server in android out dir

2.environment
	1)file prepare
		on android
			act-server
			control_tool/act-client/content
			preset file chardev-sw.set
		
		note(act-server,chardev-sw.set,content these three must in one dir)
	
	2)pc prepare
		arm_isp/driver/control_tool_files/IV009-SW-Control.xml

3.connection
	1)on android
		1.connect the network
		2.open android camera app
		3.run ./act-server --preset=chardev-sw.set
	
	2)on pc
		1.after all the set on linux, open chrome/firefox,input ip address
		http://10.18.29.119:8000/
		2.if all the settings are set, the webpage will note you to load xml, then choose IV009-SW-Control.xml
		3.you will see the isp tool window

thus,isp tool environment build complete.

