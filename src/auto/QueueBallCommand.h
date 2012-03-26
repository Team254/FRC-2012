#include "auto/AutoCommand.h"

class Shooter;
class Intake;

class QueueBallCommand : public AutoCommand {
 public:
  
  QueueBallCommand(Shooter* shooter, Intake* intake, double timeout);
  
  void Initialize();
  
  bool Run();
  
  ~QueueBallCommand();
  
   
	
};
