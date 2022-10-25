const exec = require('child_process').execSync;
const write = require('fs').writeFileSync;

function codeGenerationLoader(input) {
    var output = ""
    write('/tmp/widget_file.py', input);
    output = exec('sh ' + 'pyparse.sh', {stdio: 'pipe', 'cwd': __dirname});
    console.log(output.toString())
    return output.toString();
}

module.exports = codeGenerationLoader;
module.exports.default = codeGenerationLoader
