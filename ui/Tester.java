import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class PIDTester {
  
	public static void main(String[] args) {
		int portNumber = 41234;
		String message;
		byte[] buf;
		DatagramPacket packet;
		
		try {
			DatagramSocket socket = new DatagramSocket(); 
			InetAddress serverIP = InetAddress.getByName("127.0.0.1");
			
			for(int i = 0; true; i++) {
				message = "{\"S\":" + ((Math.sin(i) + 1) *50)
						+ ", \"V\":" + ((Math.cos(i) + 1) * 50)
						+ ", \"C\":" + Math.cos(i)
						+ "}";
						
				buf = message.getBytes();
				packet = new DatagramPacket(buf, buf.length, 
				                                serverIP, portNumber);
				socket.send(packet);
				System.out.println(i + " sent " + message);
				Thread.sleep(100);
			}
			
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

}
