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

#ifndef QS_RENDERER_KEYTIMER_H
#define QS_RENDERER_KEYTIMER_H

#include <ctime>

class keytimer
{
public:

	keytimer()
	{
		active = false;
		begin = clock();
		time_to_block = 0.0;
	}

	bool keyPressed(bool actually_pressed)
	{
		bool ret_val = actually_pressed;

		if(active == true)
		{
			clock_t end = clock();
			double elapsed_secs = float(end - begin)/CLOCKS_PER_SEC;
			if (elapsed_secs > time_to_block)
				active = false;
			
			ret_val = false;
		}
		else if(actually_pressed == true && active == false)
		{
			active = true;
			begin = clock();
		}

		return ret_val;
	}

	void set_time_to_block(unsigned int time_to_block_ms)
	{
		time_to_block = ((float) time_to_block_ms) / 1000.0f;
	}

private:

	bool active;
	clock_t begin;
	float time_to_block;
};

#endif