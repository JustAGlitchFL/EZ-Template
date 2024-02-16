"use strict";(self.webpackChunkmy_website=self.webpackChunkmy_website||[]).push([[3405],{960:(e,t,i)=>{i.r(t),i.d(t,{assets:()=>a,contentTitle:()=>o,default:()=>h,frontMatter:()=>r,metadata:()=>c,toc:()=>l});var s=i(5893),n=i(1151);const r={title:"Joystick Curves",description:"Adjusting the joystick's behavior to make it exponential"},o="Joystick Curves",c={id:"tutorials/joystick_curve",title:"Joystick Curves",description:"Adjusting the joystick's behavior to make it exponential",source:"@site/versioned_docs/version-2.x/tutorials/joystick_curve.md",sourceDirName:"tutorials",slug:"/tutorials/joystick_curve",permalink:"/EZ-Template/2.x/tutorials/joystick_curve",draft:!1,unlisted:!1,editUrl:"https://github.com/EZ-Robotics/EZ-Template/tree/website/versioned_docs/version-2.x/tutorials/joystick_curve.md",tags:[],version:"2.x",frontMatter:{title:"Joystick Curves",description:"Adjusting the joystick's behavior to make it exponential"},sidebar:"tutorialSidebar",previous:{title:"Example Autonomous Routines",permalink:"/EZ-Template/2.x/tutorials/example_autons"},next:{title:"PID Tutorial",permalink:"/EZ-Template/2.x/tutorials/pid"}},a={},l=[{value:"Introduction",id:"introduction",level:2},{value:"Enabling",id:"enabling",level:2},{value:"Disabling",id:"disabling",level:2}];function d(e){const t={a:"a",admonition:"admonition",code:"code",h1:"h1",h2:"h2",p:"p",pre:"pre",strong:"strong",...(0,n.a)(),...e.components};return(0,s.jsxs)(s.Fragment,{children:[(0,s.jsx)(t.h1,{id:"joystick-curves",children:(0,s.jsx)(t.strong,{children:"Joystick Curves"})}),"\n",(0,s.jsx)(t.h2,{id:"introduction",children:"Introduction"}),"\n",(0,s.jsxs)(t.p,{children:["Using the ",(0,s.jsx)(t.a,{href:"https://www.desmos.com/calculator/7oyvwwpmed",children:"5225 curves from 2018"}),", (explained ",(0,s.jsx)(t.a,{href:"https://www.vexforum.com/t/team-5225a-in-the-zone-code-release-yes-you-read-that-right/63199/10",children:"here"}),"). The x-axis is the joystick input and the y-axis is the motor output."]}),"\n",(0,s.jsx)(t.p,{children:"Normally, pushing the joystick half way means the robot goes half speed. With an input curve, pushing the joystick half way may only move the robot at 1/4 power. This means more of the joystick movement goes to lower speeds, giving you more control of the robot."}),"\n",(0,s.jsxs)(t.p,{children:[(0,s.jsx)(t.a,{href:"https://www.desmos.com/calculator/7oyvwwpmed",children:"This curve"})," is adjustable by changing the T value.  We can do this live on our controllers, or have it enable during ",(0,s.jsx)(t.code,{children:"initialize()"}),"."]}),"\n",(0,s.jsx)(t.p,{children:"When the robot is on, tapping/holding the left/right arrows will increase/decrease how large the curve is. When arcade is enabled, each stick will have it's own curve. The y/a buttons will increase/decrease the curve for the right stick."}),"\n",(0,s.jsx)(t.h2,{id:"enabling",children:"Enabling"}),"\n",(0,s.jsxs)(t.p,{children:["After you find values you like, in ",(0,s.jsx)(t.code,{children:"src/main.cpp"})," set ",(0,s.jsx)(t.code,{children:"chassis.set_curve_defaults(0, 0)"})," to whatever you liked! The first parameter is left stick, second is right stick.  When using tank, only the left stick value is used."]}),"\n",(0,s.jsx)(t.pre,{children:(0,s.jsx)(t.code,{className:"language-cpp",children:"void initialize() {\r\n  . . .\r\n  chassis.set_curve_default(2.1, 4.3);\r\n  . . .\r\n}\n"})}),"\n",(0,s.jsxs)(t.p,{children:["In ",(0,s.jsx)(t.code,{children:"src/main.cpp"}),", in ",(0,s.jsx)(t.code,{children:"void initialize()"}),", if ",(0,s.jsx)(t.code,{children:"chassis.toggle_modify_curve_with_controller(true)"})," is enabled, by pressing the left/right (or y/a if arcade is enabled), you can live adjust your curve and it will display to your controller!"]}),"\n",(0,s.jsx)(t.pre,{children:(0,s.jsx)(t.code,{className:"language-cpp",children:"void initialize() {\r\n  . . .\r\n  chassis.toggle_modify_curve_with_controller(true); \r\n  chassis.set_curve_default(2.1, 4.3); \r\n  . . .\r\n}\n"})}),"\n",(0,s.jsx)(t.p,{children:"If you have an sd card plugged in, after changing the number with your controller, the value will save to the sd card."}),"\n",(0,s.jsx)(t.admonition,{type:"warning",children:(0,s.jsxs)(t.p,{children:["You must remove ",(0,s.jsx)(t.code,{children:"chassis.set_curve_default(x, x)"})," from ",(0,s.jsx)(t.code,{children:"initialize()"})," if you have an SD card, otherwise this will overwrite the SD card when you power on the robot!"]})}),"\n",(0,s.jsx)(t.h2,{id:"disabling",children:"Disabling"}),"\n",(0,s.jsxs)(t.p,{children:["To disable the joystick curve entirely, in ",(0,s.jsx)(t.code,{children:"src/main.cpp"})," make sure these are 0 ",(0,s.jsx)(t.code,{children:"chassis.set_curve_defaults(0, 0)"}),".  If you don't want to modify the curve with the controller, make sure ",(0,s.jsx)(t.code,{children:"chassis.toggle_modify_curve_with_controller(false)"})," is false."]}),"\n",(0,s.jsx)(t.pre,{children:(0,s.jsx)(t.code,{className:"language-cpp",children:"void initialize() {\r\n  . . .\r\n  chassis.toggle_modify_curve_with_controller(false); \r\n  chassis.set_curve_default(0, 0); \r\n  . . .\r\n}\n"})})]})}function h(e={}){const{wrapper:t}={...(0,n.a)(),...e.components};return t?(0,s.jsx)(t,{...e,children:(0,s.jsx)(d,{...e})}):d(e)}},1151:(e,t,i)=>{i.d(t,{Z:()=>c,a:()=>o});var s=i(7294);const n={},r=s.createContext(n);function o(e){const t=s.useContext(r);return s.useMemo((function(){return"function"==typeof e?e(t):{...t,...e}}),[t,e])}function c(e){let t;return t=e.disableParentContext?"function"==typeof e.components?e.components(n):e.components||n:o(e.components),s.createElement(r.Provider,{value:t},e.children)}}}]);