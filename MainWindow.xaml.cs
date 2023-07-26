using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Media;
using LiveCharts; //Core of the library
using LiveCharts.Wpf; //The WPF controls
using MathNet.Numerics.IntegralTransforms;
using System.Numerics;
using System.Windows.Threading;

namespace MoyerLiuSerial
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : System.Windows.Window
    {
        private const int ChartNum = 100;//出现的点数
        private SerialPort serialPort;
        private byte[] buffer;
        private int count;
        private int countall = 0;
        private bool start;
        private int frameCount = 0;
        private DateTime startTime = DateTime.Now;

        private List<double> accelerationXValues = new List<double>();
        private List<double> accelerationYValues = new List<double>();
        private List<double> accelerationZValues = new List<double>();
        private SeriesCollection seriesCollection;
        private ChartValues<double> magnitudeXValues;
        private ChartValues<double> magnitudeYValues;
        private ChartValues<double> magnitudeZValues;
        private DispatcherTimer timer;
        private DispatcherTimer uiUpdateTimer;
        private Queue<Action> uiUpdateQueue;


        /// <summary>
        /// 
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
            InitializeSerialPortList();


            seriesCollection = new SeriesCollection();
            magnitudeXValues = new ChartValues<double>();
            magnitudeYValues = new ChartValues<double>();
            magnitudeZValues = new ChartValues<double>();

            seriesCollection.Add(new LineSeries
            {
                Title = "X",
                Values = magnitudeXValues,
            });
            seriesCollection.Add(new LineSeries
            {
                Title = "Y",
                Values = magnitudeYValues
            });
            seriesCollection.Add(new LineSeries
            {
                Title = "Z",
                Values = magnitudeZValues
            });

            //Axis AxisY = new Axis
            //{
            //    Title = "权重",
            //    LabelFormatter = value => value.ToString("F0"), // 设置标签格式为一位浮点
            //};

            //chart.AxisY.Clear();
            //chart.AxisY.Add(AxisY);


            chart.Series = seriesCollection;

            timer = new DispatcherTimer();
            timer.Interval = TimeSpan.FromSeconds(1);
            timer.Tick += Timer_Tick;
            timer.Start();

            uiUpdateQueue = new Queue<Action>();
            uiUpdateTimer = new DispatcherTimer();
            uiUpdateTimer.Interval = TimeSpan.FromMilliseconds(0.5);
            uiUpdateTimer.Tick += UiUpdateTimer_Tick;
            uiUpdateTimer.Start();
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Timer_Tick(object sender, EventArgs e)
        {
            if (accelerationXValues.Count < 2)
                return;

            int fftLength = 0x400;

            Complex[] fftX = new Complex[fftLength];
            Complex[] fftY = new Complex[fftLength];
            Complex[] fftZ = new Complex[fftLength];

            // 将加速度值填充到相应的复数数组中
            FillComplexArray(accelerationXValues, fftX);
            FillComplexArray(accelerationYValues, fftY);
            FillComplexArray(accelerationZValues, fftZ);

            // 对三个复数数组进行傅里叶变换
            ApplyFourierTransform(fftX);
            ApplyFourierTransform(fftY);
            ApplyFourierTransform(fftZ);

            // 计算幅度数组
            double[] magnitudeX = CalculateMagnitudeArray(fftX, fftLength);
            double[] magnitudeY = CalculateMagnitudeArray(fftY, fftLength);
            double[] magnitudeZ = CalculateMagnitudeArray(fftZ, fftLength);

            // 将前两个幅度值设置为0
            magnitudeX[0] = magnitudeY[0] = magnitudeZ[0] = 
            magnitudeX[1] = magnitudeY[1] = magnitudeZ[1] = 0;

            // 获取前150个幅度值中的最大值
            double maxMagnitudeX = GetMaxMagnitude(magnitudeX, ChartNum);
            double maxMagnitudeY = GetMaxMagnitude(magnitudeY, ChartNum);
            double maxMagnitudeZ = GetMaxMagnitude(magnitudeZ, ChartNum);

            try
            {
                // 更新UI的操作
                uiUpdateQueue.Enqueue(() =>
                {
                    UpdateMagnitudeValues(magnitudeX, magnitudeY, magnitudeZ);
                });

                // 清空加速度值数组
                ClearAccelerationValues();
            }
            catch (Exception ex)
            {
                MessageBox.Show("串口关闭错误：" + ex.Message);
            }
        }

        /// <summary>
        /// 填充复数数组
        /// </summary>
        /// <param name="sourceList"></param>
        /// <param name="targetArray"></param>
        private void FillComplexArray(List<double> sourceList,
                                      Complex[] targetArray)
        {
            for (int i = 0; i < targetArray.Length; i++)
            {
                if (i < sourceList.Count)
                    targetArray[i] = new Complex(sourceList[i], 0);
                else
                    targetArray[i] = Complex.Zero;
            }
        }

        /// <summary>
        /// 应用傅里叶变换
        /// </summary>
        /// <param name="array"></param>
        private void ApplyFourierTransform(Complex[] array)
        {
            Fourier.Forward(array, FourierOptions.Default);
        }

        /// <summary>
        /// 计算幅度数组
        /// </summary>
        /// <param name="complexArray"></param>
        /// <param name="length"></param>
        /// <returns></returns>
        private double[] CalculateMagnitudeArray(Complex[] complexArray, int length)
        {
            double[] magnitudeArray = new double[length / 2];
            for (int i = 0; i < length / 2; i++)
            {
                magnitudeArray[i] = complexArray[i].Magnitude;
            }
            return magnitudeArray;
        }

        /// <summary>
        /// 获取幅度数组中的最大值
        /// </summary>
        /// <param name="magnitudeArray"></param>
        /// <param name="count"></param>
        /// <returns></returns>
        private double GetMaxMagnitude(double[] magnitudeArray, int count)
        {
            return magnitudeArray.Take(count).Max();
        }

        /// <summary>
        /// 更新幅度值列表
        /// </summary>
        /// <param name="magnitudeX"></param>
        /// <param name="magnitudeY"></param>
        /// <param name="magnitudeZ"></param>
        private void UpdateMagnitudeValues(double[] magnitudeX, double[] magnitudeY, double[] magnitudeZ)
        {
            magnitudeXValues.Clear();
            magnitudeYValues.Clear();
            magnitudeZValues.Clear();

            int chartNum = Math.Min(ChartNum, magnitudeX.Length);
            for (int i = 0; i < chartNum; i++)
            {
                magnitudeXValues.Add(magnitudeX[i]);
                magnitudeYValues.Add(magnitudeY[i]);
                magnitudeZValues.Add(magnitudeZ[i]);
            }
        }

        /// <summary>
        /// 清空加速度值列表
        /// </summary>
        private void ClearAccelerationValues()
        {
            accelerationXValues.Clear();
            accelerationYValues.Clear();
            accelerationZValues.Clear();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void UiUpdateTimer_Tick(object sender, EventArgs e)
        {

            if (uiUpdateQueue.Count > 0)
            {
                var action = uiUpdateQueue.Dequeue();
                action?.Invoke();
            }
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="n"></param>
        /// <returns></returns>
        private int GetNextPowerOfTwo(int n)
        {
            int power = 0;
            while ((1 << power) < n)
                power++;
            return 1 << power;
        }


        /// <summary>
        /// 初始化串口，获取串口列表
        /// </summary>
        private void InitializeSerialPortList()
        {
            var portlist = SerialPort.GetPortNames();
            cbPort.ItemsSource = portlist;
            cbPort.SelectedIndex = 0;
        }


        /// <summary>
        /// 按钮触发开启串口
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtOpenSerialPort_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                if (CloseSerialPort())
                {
                    BtOpenSerialPort.Content = "打开串口";
                    BtOpenSerialPort.Background = Brushes.Red;
                }
            }
            else
            {
                if (OpenSerialPort())
                {
                    BtOpenSerialPort.Content = "关闭串口";
                    BtOpenSerialPort.Background = Brushes.Green;
                }
            }
        }


        /// <summary>
        /// 关闭串口方法
        /// </summary>
        /// <returns></returns>
        private bool CloseSerialPort()
        {
            try
            {
                serialPort.Close();
                return true;
            }
            catch (Exception ex)
            {
                MessageBox.Show("串口关闭错误：" + ex.Message);
                return false;
            }
        }


        /// <summary>
        /// 开启串口方法
        /// </summary>
        /// <returns></returns>
        private bool OpenSerialPort()
        {
            try
            {
                serialPort = new SerialPort
                {
                    PortName = cbPort.Text,                      //串口名
                    DataBits = int.Parse(cbDataBits.Text),       //数据位
                    StopBits = (StopBits)Enum.Parse(typeof(StopBits), cbStopBits.Text),//停止位
                    Parity = (Parity)Enum.Parse(typeof(Parity), cbParity.Text),//校验位
                    BaudRate = Convert.ToInt32(cbBaudRate.Text)//波特率
                };
                serialPort.Open();//开串口
                serialPort.DataReceived += SerialPort_DataReceived;
                buffer = new byte[6];
                count = 0;
                start = false;

                return true;
            }
            catch (Exception ex)
            {
                MessageBox.Show("串口打开错误：" + ex.Message);
                return false;
            }
        }


        /// <summary>
        /// 接收串口数据，并处理（校验帧头
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string dataString = ReadSerialPortData();

                while (dataString.Length > 0)
                {
                    byte data = (byte)dataString[0];
                    dataString = dataString.Remove(0, 1);

                    if (count == 0)
                    {
                        if ((data >> 4) == 1)
                        {
                            start = true;
                            buffer[count] = data;
                            count++;
                        }
                        else
                        {
                            start = false;
                        }
                    }
                    else if (start)
                    {
                        buffer[count] = data;
                        count++;
                        if (count == 6)
                        {
                            start = false;
                            ProcessData(buffer);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("数据接收错误：" + ex.Message);
            }
        }


        /// <summary>
        /// 读取串口缓存数据
        /// </summary>
        /// <returns></returns>
        private string ReadSerialPortData()
        {
            StringBuilder dataString = new StringBuilder();
            while (serialPort.BytesToRead > 0)
            {
                dataString.Append(serialPort.ReadExisting());
            }
            return dataString.ToString();
        }


        /// <summary>
        /// 数据处理，
        /// </summary>
        /// <param name="buffer"></param>
        private void ProcessData(byte[] buffer)
        {
            int value1 = ((buffer[0] << 8) | buffer[1]) & 0x0FFF;//删除校验头（前4位
            int value2 = ((buffer[2] << 8) | buffer[3]) & 0x0FFF;
            int value3 = ((buffer[4] << 8) | buffer[5]) & 0x0FFF;

            double acceleration1 = ConvertToAcceleration(value1);
            double acceleration2 = ConvertToAcceleration(value2);
            double acceleration3 = ConvertToAcceleration(value3);

            accelerationXValues.Add(acceleration1);
            accelerationYValues.Add(acceleration2);
            accelerationZValues.Add(acceleration3);

            if (countall % 200 == 0)
            {

                Dispatcher.BeginInvoke(() =>
                {
                    txlReceive.Content = $"X: {acceleration1:F2}g, Y: {acceleration2:F2}g, Z: {acceleration3:F2}g";
                });
            }

            count = 0;
            countall++;

            FrameCalc();
        }


        /// <summary>
        /// 计算每秒钟发送的帧数（一帧六个字节）
        /// </summary>
        private void FrameCalc()
        {
            frameCount++;
            TimeSpan elapsed = DateTime.Now - startTime;
            if (elapsed.TotalSeconds >= 1)
            {
                double fps = frameCount / elapsed.TotalSeconds;
                frameCount = 0;
                startTime = DateTime.Now;

                uiUpdateQueue.Enqueue(() =>
                {
                    fpsLabel.Content = "Frame Rate: " + fps.ToString("F2") + " fps";
                });
            }
        }


        /// <summary>
        /// 量化传感器参数（12bit -> 0~Vcc ）
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        private double ConvertToAcceleration(int value)
        {
            double acceleration = (value * 3.3 / 4096 - 1.65) / 0.33;
            return acceleration;
        }
    }

}
