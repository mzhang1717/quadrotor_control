
class Sample
{
public:
  static int uniform(int from, int to);

  static double uniform();
  
  static double uniform(double from, double to);

  static double gaussian(double sigma);
  
  static void setRealSeed(int seed);
  
  static void setIntSeed(int seed);
};
