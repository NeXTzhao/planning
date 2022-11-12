# [配置教程](https://danishshakeel.me/configure-logitech-mx-master-3-on-linux-logiops/)

## 安装教程

```bash
## 安装依赖库
sudo apt install cmake libevdev-dev libudev-dev libconfig++-dev 

##编译源文件
git clone https://github.com/PixlOne/logiops.git
cd logiops
mkdir build && cd build
cmake ..
make -j8
sudo make install
```

## 启动

```bash
// 设置开机自启
sudo systemctl enable logid

// 修改配置文件后重新启动
sudo service logid restart

// 在build文件中运行
sudo logid
```

## logid.cfg

> 位置在：~/etc/logid.cfg 没有这个文件的话自己建一个

```html
// Logiops (Linux driver) configuration for Logitech MX Master 3.
// Includes gestures, smartshift, DPI.
// Tested on logid v0.2.3 - GNOME 3.38.4 on Zorin OS 16 Pro
// What's working:
//   1. Window snapping using Gesture button (Thumb)
//   2. Forward Back Buttons
//   3. Top button (Ratchet-Free wheel)
// What's not working:
//   1. Thumb scroll (H-scroll)
//   2. Scroll button

// File location: /etc/logid.cfg

devices: ({
  name: "Wireless Mouse MX Master 3";

  smartshift: {
    on: true;
    threshold: 15;
  };

  hiresscroll: {
    hires: true;
    invert: false;
    target: false;
  };

  dpi: 2500; // max=4000

  buttons: (
    // Forward button
    {
      cid: 0x56;
      action = {
        type: "Gestures";
        gestures: (
          {
            direction: "None";
            mode: "OnRelease";
            action = {
              type: "Keypress";
              keys: [ "KEY_LEFTALT","KEY_LEFT" ];
            }
          },
          {
            direction: "Up";
            mode: "OnRelease";
            action = {
              type: "Keypress";
              keys: [ "KEY_LEFTCTRL","KEY_LEFTALT","KEY_T" ];
            }
          },

           {
            direction: "Down";
            mode: "OnRelease";
            action = {
              type: "Keypress";
              keys: [ "KEY_LEFTMETA" ];
            }
          },

          {
            direction: "Right";
            mode: "OnRelease";
            action = {
              type: "Keypress";
              keys: [ "KEY_NEXTSONG" ];
            }
          },

          {
            direction: "Left";
            mode: "OnRelease";
            action = {
              type: "Keypress";
              keys: [ "KEY_PREVIOUSSONG" ];
            }
          }
        );
      };
    },

    // Back button
    {   
     
      cid: 0x53;
      action = {
        type: "Gestures";
      	gestures: (
	{
	  direction: "None";
	  mode: "OnRelease";
	  action = {
	    type: "Keypress";
	    keys: [ "KEY_LEFTALT","KEY_RIGHT" ];
	  }
        },
        
  	{
	    direction: "Down";
	    mode: "OnRelease";
	    action = {
	      type: "Keypress";
	      keys: [ "KEY_LEFTCTRL","KEY_C" ];
	    }
        }
      );
    };
    },

    // Gesture button (hold and move)
    {
      cid: 0xc3;
      action = {
        type: "Gestures";
        gestures: (
          {
            direction: "None";
            mode: "OnRelease";
            action = {
              type: "Keypress";
              keys: [ "KEY_LEFTMETA" ]; // open activities overview
            }
          },

          {
            direction: "Right";
            mode: "OnRelease";
            action = {
              type: "Keypress";
              keys: [ "KEY_LEFTMETA", "KEY_RIGHT" ]; // snap window to right
            }
          },

          {
            direction: "Left";
            mode: "OnRelease";
            action = {
              type: "Keypress";
              keys: [ "KEY_LEFTMETA", "KEY_LEFT" ];
            }
		  },

		  {
            direction: "Up";
            mode: "onRelease";
            action = {
              type: "Keypress";
              keys: [ "KEY_LEFTMETA", "KEY_UP" ]; // maximize window
            }
		  },
		  
		  {
            direction: "Down";
            mode: "OnRelease";
            action = {
              type: "Keypress";
              keys: [ "KEY_LEFTMETA", "KEY_DOWN" ]; // minimize window
            }
          }
        );
      };
    },
	
    // Top button
    {
      cid: 0xc4;
      action = {
        type: "Gestures";
        gestures: (
          {
            direction: "None";
            mode: "OnRelease";
            action = {
              type: "ToggleSmartShift";
            }
          },

          {
            direction: "Up";
            mode: "OnRelease";
            action = {
              type: "ChangeDPI";
              inc: 1000,
            }
          },

          {
            direction: "Down";
            mode: "OnRelease";
            action = {
              type: "ChangeDPI";
              inc: -1000,
            }
          }
        );
      };
    }
  );
});
```

