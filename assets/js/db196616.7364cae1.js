"use strict";(self.webpackChunkmy_website=self.webpackChunkmy_website||[]).push([[1842],{3227:(e,r,l)=>{l.r(r),l.d(r,{assets:()=>d,contentTitle:()=>i,default:()=>p,frontMatter:()=>o,metadata:()=>c,toc:()=>u});var t=l(5893),n=l(1151),s=l(4866),a=l(5162);const o={layout:"default",title:"User Control",parent:"Docs",description:"",nav_order:2},i="User Control",c={id:"docs/user_control",title:"User Control",description:"",source:"@site/versioned_docs/version-2.x/docs/user_control.md",sourceDirName:"docs",slug:"/docs/user_control",permalink:"/EZ-Template/2.x/docs/user_control",draft:!1,unlisted:!1,editUrl:"https://github.com/EZ-Robotics/EZ-Template/tree/website/versioned_docs/version-2.x/docs/user_control.md",tags:[],version:"2.x",frontMatter:{layout:"default",title:"User Control",parent:"Docs",description:"",nav_order:2},sidebar:"tutorialSidebar",previous:{title:"Drive and Telemetry",permalink:"/EZ-Template/2.x/docs/set_and_get_drive"},next:{title:"Util",permalink:"/EZ-Template/2.x/docs/util"}},d={},u=[{value:"Drive Modes",id:"drive-modes",level:2},{value:"tank()",id:"tank",level:3},{value:"arcade_standard()",id:"arcade_standard",level:3},{value:"arcade_flipped()",id:"arcade_flipped",level:3},{value:"Joystick Functions",id:"joystick-functions",level:2},{value:"initialize()",id:"initialize",level:3},{value:"init_curve_sd()",id:"init_curve_sd",level:3},{value:"set_curve_defaults()",id:"set_curve_defaults",level:3},{value:"set_active_brake()",id:"set_active_brake",level:3},{value:"toggle_modify_curve_with_controller()",id:"toggle_modify_curve_with_controller",level:3},{value:"set_left_curve_buttons()",id:"set_left_curve_buttons",level:3},{value:"set_right_curve_buttons()",id:"set_right_curve_buttons",level:3},{value:"left_curve_function()",id:"left_curve_function",level:3},{value:"right_curve_function()",id:"right_curve_function",level:3},{value:"set_joystick_threshold()",id:"set_joystick_threshold",level:3},{value:"joy_thresh_opcontrol()",id:"joy_thresh_opcontrol",level:3},{value:"modify_curve_with_controller()",id:"modify_curve_with_controller",level:3}];function h(e){const r={a:"a",br:"br",code:"code",h1:"h1",h2:"h2",h3:"h3",p:"p",pre:"pre",strong:"strong",...(0,n.a)(),...e.components};return(0,t.jsxs)(t.Fragment,{children:[(0,t.jsx)(r.h1,{id:"user-control",children:(0,t.jsx)(r.strong,{children:"User Control"})}),"\n",(0,t.jsx)(r.h2,{id:"drive-modes",children:"Drive Modes"}),"\n",(0,t.jsx)(r.h3,{id:"tank",children:"tank()"}),"\n",(0,t.jsx)(r.p,{children:"Sets the drive to the left and right y axis."}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex15",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void opcontrol() {\r\n  while (true) {\r\n    chassis.tank();\r\n    \r\n    pros::delay(ez::util::DELAY_TIME);\r\n  }\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void tank();\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"arcade_standard",children:"arcade_standard()"}),"\n",(0,t.jsx)(r.p,{children:"Sets the drive to standard arcade.  Left stick is fwd/rev."}),"\n",(0,t.jsxs)(r.p,{children:[(0,t.jsx)(r.code,{children:"stick_type"})," is either ",(0,t.jsx)(r.code,{children:"ez::SPLIT"})," or ",(0,t.jsx)(r.code,{children:"ez::SINGLE"})]}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex1",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void opcontrol() {\r\n  while (true) {\r\n    chassis.arcade_standard(ez::SPIT); // For split arcade\r\n    // chassis.arcade_standard(ez::SINGLE); // For single arcade\r\n    \r\n    pros::delay(ez::util::DELAY_TIME);\r\n  }\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void arcade_standard(e_type stick_type);\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"arcade_flipped",children:"arcade_flipped()"}),"\n",(0,t.jsx)(r.p,{children:"Sets the drive to flipped arcade.  Right stick is fwd/rev."}),"\n",(0,t.jsxs)(r.p,{children:[(0,t.jsx)(r.code,{children:"stick_type"})," is either ",(0,t.jsx)(r.code,{children:"ez::SPLIT"})," or ",(0,t.jsx)(r.code,{children:"ez::SINGLE"})]}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex2",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void opcontrol() {\r\n  while (true) {\r\n    chassis.arcade_flipped(ez::SPIT); // For split arcade\r\n    // chassis.arcade_flipped(ez::SINGLE); // For single arcade\r\n    \r\n    pros::delay(ez::util::DELAY_TIME);\r\n  }\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void arcade_flipped(e_type stick_type);\n"})})})]}),"\n",(0,t.jsx)(r.h2,{id:"joystick-functions",children:"Joystick Functions"}),"\n",(0,t.jsx)(r.h3,{id:"initialize",children:"initialize()"}),"\n",(0,t.jsxs)(r.p,{children:["Runs ",(0,t.jsx)(r.code,{children:"init_curve_sd()"})," and ",(0,t.jsx)(r.code,{children:"imu_calibrate()"}),"."]}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex3",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void initialize() {\r\n  chassis.initialize();\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void Drive::initialize();\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"init_curve_sd",children:"init_curve_sd()"}),"\n",(0,t.jsx)(r.p,{children:"Sets the left/right curve constants to what's on the SD card.  If the SD card is empty, creates needed files."}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex4",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void initialize() {\r\n  chassis.init_curve_sd();\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void init_curve_sd();\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"set_curve_defaults",children:"set_curve_defaults()"}),"\n",(0,t.jsx)(r.p,{children:"Sets the left/right curve defaults and saves new values to the SD card."}),"\n",(0,t.jsxs)(r.p,{children:[(0,t.jsx)(r.code,{children:"left"})," left input curve",(0,t.jsx)(r.br,{}),"\n",(0,t.jsx)(r.code,{children:"right"})," right input curve"]}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex5",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void initialize() {\r\n  chassis.set_curve_defaults(2, 2);\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void set_curve_default(double left, double right);\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"set_active_brake",children:"set_active_brake()"}),"\n",(0,t.jsx)(r.p,{children:"Active brake runs a P loop on the drive when joysticks are within their threshold."}),"\n",(0,t.jsxs)(r.p,{children:[(0,t.jsx)(r.code,{children:"kp"})," proportional constant for drive"]}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex6",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void initialize() {\r\n  chassis.set_active_brake(0.1);\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void set_active_brake(double kp);\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"toggle_modify_curve_with_controller",children:"toggle_modify_curve_with_controller()"}),"\n",(0,t.jsx)(r.p,{children:"Enables/disables buttons used for modifying the controller curve with the joystick."}),"\n",(0,t.jsxs)(r.p,{children:[(0,t.jsx)(r.code,{children:"toggle"})," true enables, false disables"]}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex7",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void initialize() {\r\n  chassis.toggle_modify_curve_with_controller(true);\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void toggle_modify_curve_with_controller(bool toggle);\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"set_left_curve_buttons",children:"set_left_curve_buttons()"}),"\n",(0,t.jsx)(r.p,{children:"Sets the buttons that are used to modify the left input curve.  The example is the default."}),"\n",(0,t.jsxs)(r.p,{children:[(0,t.jsx)(r.code,{children:"decrease"})," a pros button",(0,t.jsx)(r.br,{}),"\n",(0,t.jsx)(r.code,{children:"increase"})," a pros button"]}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex8",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void initialize() {\r\n  chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void set_left_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"set_right_curve_buttons",children:"set_right_curve_buttons()"}),"\n",(0,t.jsx)(r.p,{children:"Sets the buttons that are used to modify the right input curve.  The example is the default."}),"\n",(0,t.jsxs)(r.p,{children:[(0,t.jsx)(r.code,{children:"decrease"})," a pros button",(0,t.jsx)(r.br,{}),"\n",(0,t.jsx)(r.code,{children:"increase"})," a pros button"]}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex9",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void initialize() {\r\n  chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void set_right_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"left_curve_function",children:"left_curve_function()"}),"\n",(0,t.jsxs)(r.p,{children:["Returns the input times the curve ",(0,t.jsx)(r.a,{href:"https://www.desmos.com/calculator/7oyvwwpmed",children:"here"}),".  ",(0,t.jsx)(r.code,{children:"tank()"}),", ",(0,t.jsx)(r.code,{children:"arcade_standard()"}),", and ",(0,t.jsx)(r.code,{children:"arcade_flipped()"})," all handle this for you.  When tank is enabled, only this curve is used."]}),"\n",(0,t.jsxs)(r.p,{children:[(0,t.jsx)(r.code,{children:"x"})," input value"]}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex10",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void opcontrol() {\r\n  while (true) {\r\n    int l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));\r\n    int r_stick = left_curve_function(master.get_analog(ANALOG_RIGHT_Y));\r\n    \r\n    chassis.set_tank(l_stick, r_stick);\r\n    \r\n    pros::delay(ez::util::DELAY_TIME);\r\n  }\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"double left_curve_function(double x);\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"right_curve_function",children:"right_curve_function()"}),"\n",(0,t.jsxs)(r.p,{children:["Returns the input times the curve ",(0,t.jsx)(r.a,{href:"https://www.desmos.com/calculator/7oyvwwpmed",children:"here"}),".  ",(0,t.jsx)(r.code,{children:"tank()"}),", ",(0,t.jsx)(r.code,{children:"arcade_standard()"}),", and ",(0,t.jsx)(r.code,{children:"arcade_flipped()"})," all handle this for you."]}),"\n",(0,t.jsxs)(r.p,{children:[(0,t.jsx)(r.code,{children:"x"})," input value"]}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex11",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void opcontrol() {\r\n  while (true) {\r\n    int l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));\r\n    int r_stick = left_curve_function(master.get_analog(ANALOG_RIGHT_Y));\r\n    \r\n    chassis.set_tank(l_stick, r_stick);\r\n    \r\n    pros::delay(ez::util::DELAY_TIME);\r\n  }\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"double right_curve_function(double x);\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"set_joystick_threshold",children:"set_joystick_threshold()"}),"\n",(0,t.jsx)(r.p,{children:"Threshold the joystick will return 0 within.  This is useful because not all joysticks will return perfectly to 0 when let go."}),"\n",(0,t.jsxs)(r.p,{children:[(0,t.jsx)(r.code,{children:"threshold"})," an integer, recommended to be less then 5"]}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex12",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void initialize() {\r\n  chassis.set_joystick_threshold(5);\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void set_joystick_threshold(int threshold);\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"joy_thresh_opcontrol",children:"joy_thresh_opcontrol()"}),"\n",(0,t.jsxs)(r.p,{children:["Runs the joystick control.  Sets the left drive to ",(0,t.jsx)(r.code,{children:"l_stick"}),", and right drive to ",(0,t.jsx)(r.code,{children:"r_stick"}),".  Runs active brake and joystick thresholds."]}),"\n",(0,t.jsxs)(r.p,{children:[(0,t.jsx)(r.code,{children:"l_stick"})," left joystick value\r\n",(0,t.jsx)(r.code,{children:"r_stick"})," right joystick value"]}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex13",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void opcontrol() {\r\n  while (true) {\r\n    chassis.joy_thresh_opcontroL(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));\r\n\r\n    pros::delay(ez::util::DELAY_TIME);\r\n  }\r\n  chassis.set_joystick_threshold(5);\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void joy_thresh_opcontrol(int l_stick, int r_stick);\n"})})})]}),"\n",(0,t.jsx)(r.h3,{id:"modify_curve_with_controller",children:"modify_curve_with_controller()"}),"\n",(0,t.jsx)(r.p,{children:"Allows the user to modify the curve with the controller."}),"\n",(0,t.jsxs)(s.Z,{groupId:"ex14",defaultValue:"proto",values:[{label:"Prototype",value:"proto"},{label:"Example",value:"example"}],children:[(0,t.jsx)(a.Z,{value:"example",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void opcontrol() {\r\n  while (true) {\r\n    chassis.joy_thresh_opcontroL(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));\r\n\r\n    chassis.modify_curve_with_controller();\r\n\r\n    pros::delay(ez::util::DELAY_TIME);\r\n  }\r\n  chassis.set_joystick_threshold(5);\r\n}\n"})})}),(0,t.jsx)(a.Z,{value:"proto",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-cpp",children:"void modify_curve_with_controller();\n"})})})]})]})}function p(e={}){const{wrapper:r}={...(0,n.a)(),...e.components};return r?(0,t.jsx)(r,{...e,children:(0,t.jsx)(h,{...e})}):h(e)}},5162:(e,r,l)=>{l.d(r,{Z:()=>a});l(7294);var t=l(6905);const n={tabItem:"tabItem_Ymn6"};var s=l(5893);function a(e){let{children:r,hidden:l,className:a}=e;return(0,s.jsx)("div",{role:"tabpanel",className:(0,t.Z)(n.tabItem,a),hidden:l,children:r})}},4866:(e,r,l)=>{l.d(r,{Z:()=>y});var t=l(7294),n=l(6905),s=l(2466),a=l(6550),o=l(469),i=l(1980),c=l(7392),d=l(12);function u(e){return t.Children.toArray(e).filter((e=>"\n"!==e)).map((e=>{if(!e||(0,t.isValidElement)(e)&&function(e){const{props:r}=e;return!!r&&"object"==typeof r&&"value"in r}(e))return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))?.filter(Boolean)??[]}function h(e){const{values:r,children:l}=e;return(0,t.useMemo)((()=>{const e=r??function(e){return u(e).map((e=>{let{props:{value:r,label:l,attributes:t,default:n}}=e;return{value:r,label:l,attributes:t,default:n}}))}(l);return function(e){const r=(0,c.l)(e,((e,r)=>e.value===r.value));if(r.length>0)throw new Error(`Docusaurus error: Duplicate values "${r.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(e),e}),[r,l])}function p(e){let{value:r,tabValues:l}=e;return l.some((e=>e.value===r))}function _(e){let{queryString:r=!1,groupId:l}=e;const n=(0,a.k6)(),s=function(e){let{queryString:r=!1,groupId:l}=e;if("string"==typeof r)return r;if(!1===r)return null;if(!0===r&&!l)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return l??null}({queryString:r,groupId:l});return[(0,i._X)(s),(0,t.useCallback)((e=>{if(!s)return;const r=new URLSearchParams(n.location.search);r.set(s,e),n.replace({...n.location,search:r.toString()})}),[s,n])]}function v(e){const{defaultValue:r,queryString:l=!1,groupId:n}=e,s=h(e),[a,i]=(0,t.useState)((()=>function(e){let{defaultValue:r,tabValues:l}=e;if(0===l.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(r){if(!p({value:r,tabValues:l}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${r}" but none of its children has the corresponding value. Available values are: ${l.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return r}const t=l.find((e=>e.default))??l[0];if(!t)throw new Error("Unexpected error: 0 tabValues");return t.value}({defaultValue:r,tabValues:s}))),[c,u]=_({queryString:l,groupId:n}),[v,x]=function(e){let{groupId:r}=e;const l=function(e){return e?`docusaurus.tab.${e}`:null}(r),[n,s]=(0,d.Nk)(l);return[n,(0,t.useCallback)((e=>{l&&s.set(e)}),[l,s])]}({groupId:n}),j=(()=>{const e=c??v;return p({value:e,tabValues:s})?e:null})();(0,o.Z)((()=>{j&&i(j)}),[j]);return{selectedValue:a,selectValue:(0,t.useCallback)((e=>{if(!p({value:e,tabValues:s}))throw new Error(`Can't select invalid tab value=${e}`);i(e),u(e),x(e)}),[u,x,s]),tabValues:s}}var x=l(2389);const j={tabList:"tabList__CuJ",tabItem:"tabItem_LNqP"};var m=l(5893);function f(e){let{className:r,block:l,selectedValue:t,selectValue:a,tabValues:o}=e;const i=[],{blockElementScrollPositionUntilNextRender:c}=(0,s.o5)(),d=e=>{const r=e.currentTarget,l=i.indexOf(r),n=o[l].value;n!==t&&(c(r),a(n))},u=e=>{let r=null;switch(e.key){case"Enter":d(e);break;case"ArrowRight":{const l=i.indexOf(e.currentTarget)+1;r=i[l]??i[0];break}case"ArrowLeft":{const l=i.indexOf(e.currentTarget)-1;r=i[l]??i[i.length-1];break}}r?.focus()};return(0,m.jsx)("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,n.Z)("tabs",{"tabs--block":l},r),children:o.map((e=>{let{value:r,label:l,attributes:s}=e;return(0,m.jsx)("li",{role:"tab",tabIndex:t===r?0:-1,"aria-selected":t===r,ref:e=>i.push(e),onKeyDown:u,onClick:d,...s,className:(0,n.Z)("tabs__item",j.tabItem,s?.className,{"tabs__item--active":t===r}),children:l??r},r)}))})}function g(e){let{lazy:r,children:l,selectedValue:n}=e;const s=(Array.isArray(l)?l:[l]).filter(Boolean);if(r){const e=s.find((e=>e.props.value===n));return e?(0,t.cloneElement)(e,{className:"margin-top--md"}):null}return(0,m.jsx)("div",{className:"margin-top--md",children:s.map(((e,r)=>(0,t.cloneElement)(e,{key:r,hidden:e.props.value!==n})))})}function b(e){const r=v(e);return(0,m.jsxs)("div",{className:(0,n.Z)("tabs-container",j.tabList),children:[(0,m.jsx)(f,{...e,...r}),(0,m.jsx)(g,{...e,...r})]})}function y(e){const r=(0,x.Z)();return(0,m.jsx)(b,{...e,children:u(e.children)},String(r))}},1151:(e,r,l)=>{l.d(r,{Z:()=>o,a:()=>a});var t=l(7294);const n={},s=t.createContext(n);function a(e){const r=t.useContext(s);return t.useMemo((function(){return"function"==typeof e?e(r):{...r,...e}}),[r,e])}function o(e){let r;return r=e.disableParentContext?"function"==typeof e.components?e.components(n):e.components||n:a(e.components),t.createElement(s.Provider,{value:r},e.children)}}}]);