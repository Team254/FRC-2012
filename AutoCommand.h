class AutoCommand {
 public:
  virtual void Initialize();
  virtual bool Run() = 0;
  virtual ~AutoCommand();
};

