/*

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef QC_MATHHELP_H
#define QC_MATHHELP_H

#define DEG2RAD(x)	(x)*0.017453292
#define RAD2DEG(x)	(x)*57.29577951
#define Pi			3.1415926535897
#define Pi2			6.28318530718

#define wrap_Pi(x) (x < -Pi ? x+Pi2 : (x > Pi ? x - Pi2: x))
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

template<typename T>
T mapp(T input, T in_min, T in_max, T out_min, T out_max)
{
	return T((input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

template<typename T>
T constrainn(T in, T minimum, T maximum)
{
	in = (in <= minimum) ? minimum : in;
	in = (in >= maximum) ? maximum : in;

	return in;
}

template<typename T>
T deadband(T value, const T threshold)
{
	// typecast of 0?
	if((value > 0 && value < threshold) || (value < 0 && value > -threshold))
		value = 0;
	else if(value > 0)
		value -= threshold;
	else if(value < 0)
		value += threshold;

	return value;
}

template<typename T>
T smoothLinear(T value, const T minVal, const T maxVal, const T offset)
{
	T border = minVal + offset;
	T range = maxVal - minVal;
	
	// values don't make any sense
	if(border >= maxVal)
		return 0;			// typecast of 0?
	
	if(value <= border)
		return 0;			// typecast of 0?
	
	if(value >= maxVal)
		return 1;			// typecast of 1?	
	
	value -= border;
	
	value /= range-offset;

	return value;
}

/* Ring buffer template class.
 * I don't want to use heap memory (malloc, realloc etc.). 
 * Therefore stack memory can be allocated statically and coupled via 'T *ary' and 'int bufLen'.
 * Through these variables the static memory is read and written in a ring buffer style.
 */
template<class T>
class ringBuffer
{
	public:
	
		ringBuffer();
		ringBuffer(T *ary_arg, int bufLen_arg);
		~ringBuffer();
		
		// ring buffer access functions, return true on success
		void pushNewElem(T newElem);
		bool getNthElem(T &out, int n);		// latest element has index 0, then comes 1, 2, 3 ... bufLen-1
		bool getHeadElem(T &out);
		bool getPreHeadElem(T &out); 	
		bool getTailElem(T &out);

	private:
	
		T *ary;
		int bufLen;
		int idx;
};

template <class T>
ringBuffer<T>::ringBuffer()
{
	ary = 0;
	bufLen = 0;
	idx = 0;
}

template <class T>
ringBuffer<T>::ringBuffer(T *ary_arg, int bufLen_arg)
{
	ary = ary_arg;
	bufLen = bufLen_arg;
	idx = 0;
}

template <class T>
ringBuffer<T>::~ringBuffer()
{
}

template <class T>
void ringBuffer<T>::pushNewElem(T newElem)
{
	ary[idx] = newElem;
	idx = (idx + 1) % bufLen;	
}

template <class T>
bool ringBuffer<T>::getNthElem(T &out, int n)
{
	// check if n is valid
	if(n < 0 || n >= bufLen)
		return false;
	
	// n = 0 means the head element, n = bufLen-1 is the tail element.
	// idx does NOT point to the latest element but to the next index to write to.
	int nthElem = idx - (n+1);
	
	// true modulo
	nthElem = (nthElem < 0) ? nthElem + bufLen : nthElem;
	
	out = ary[nthElem];
	
	return true;
}

template <class T>
bool ringBuffer<T>::getHeadElem(T &out)
{
	return getNthElem(out, 0);
}

template <class T>
bool ringBuffer<T>::getPreHeadElem(T &out)
{
	return getNthElem(out, 1);
}  

template <class T>
bool ringBuffer<T>::getTailElem(T &out)
{
	return getNthElem(out, bufLen-1);
}  

#endif