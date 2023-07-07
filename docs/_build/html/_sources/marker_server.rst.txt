Marker Server Node
-----------------------

This node implements a service to provide information of each rooms id

Services:
	/room_info ---> RoomInformation (Request, Response)

Description:

This node implements a service: it requires the id (marker) detected by the 
robot and it replies with the information about the corresponding room (name of
the room, coordinates of the center, connections with other rooms)
Additionaly uses Callback function for ``room_info`` service.

