const {chromium} = require('/home/youran/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
(async()=>{
 const browser=await chromium.launch({headless:true,executablePath:"/opt/google/chrome/chrome",args:['--no-sandbox','--autoplay-policy=no-user-gesture-required']});
 try{
 const page=await browser.newPage({viewport:{width:1400,height:1000}});
 const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto('http://127.0.0.1:18765/preview');
 await page.waitForFunction(()=>{const v=document.querySelector('video');return v.videoWidth===1280&&v.currentTime>0.2;},{},{timeout:25000});
 console.log(await page.locator('#status').innerText());
 await page.click('#reconnect');
 await page.waitForFunction(()=>{const v=document.querySelector('video');return v.videoWidth===1280&&v.currentTime>0.2;},{},{timeout:25000});
 console.log('Reconnect passed; browser errors:',errors);
 if(errors.length)throw Error(errors.join('\n'));
 await page.screenshot({path:'work/preview.png'});
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
