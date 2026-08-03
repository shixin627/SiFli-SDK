#!/usr/bin/env node
/*
 * Sign a Skai app package — the publisher side of skaiapp-manifest.schema.json v1.
 *
 * Node rather than Python, which is what the rest of tools/sdk uses: signing
 * needs ECDSA P-256 and neither `cryptography` nor `ecdsa` is installed here,
 * while Node's WebCrypto ships it and returns the signature as raw r||s — the
 * exact form the schema asks for, with no DER unwrapping step to get wrong.
 *
 *   node sign_pkg.js --payload app.js --type js --id calc --name Calculator \
 *                    --version 1.0.0 --caps ui.label,ui.button --out DIR
 *
 * Writes <out>/manifest.json, <out>/payload and, unless --key is given, a fresh
 * <out>/key.json. The key file is the publisher's identity: losing it means
 * never being able to update the app again (TOFU), which is the point.
 *
 * --tamper <what> deliberately breaks one property AFTER signing, for the
 * firmware's hostile-input tests: payload | caps | keyid | sig | version.
 */
'use strict';

const fs = require('fs');
const path = require('path');
const { webcrypto } = require('crypto');
const { subtle } = webcrypto;

const b64u = (buf) => Buffer.from(buf).toString('base64')
    .replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');

function args(argv)
{
    const o = {};
    for (let i = 2; i < argv.length; i++)
    {
        if (!argv[i].startsWith('--')) continue;
        const k = argv[i].slice(2);
        const v = (argv[i + 1] && !argv[i + 1].startsWith('--')) ? argv[++i] : 'true';
        o[k] = v;
    }
    return o;
}

async function sha256(bytes)
{
    return Buffer.from(await subtle.digest('SHA-256', bytes));
}

/* The one thing that must match the firmware byte for byte. Defined in the
   schema's definitions.signedInput: LF-joined, trailing LF, all ASCII, and
   capabilities sorted bytewise so the order in the manifest cannot change the
   digest. */
function signedInput(m)
{
    const caps = [...m.capabilities].sort().join(',');
    return Buffer.from(
        'skaiapp-v1\n' +
        m.publisher.keyid + '\n' +
        m.app.id + '\n' +
        m.app.version + '\n' +
        m.payload.type + '\n' +
        String(m.payload.size) + '\n' +
        m.payload.sha256 + '\n' +
        caps + '\n', 'ascii');
}

async function main()
{
    const a = args(process.argv);
    if (!a.payload || !a.id || !a.out)
    {
        console.error('usage: sign_pkg.js --payload FILE --id ID --out DIR '
                      + '[--type js|declarative] [--name N] [--version 1.0.0] '
                      + '[--caps a.b,c.d] [--skai ">=1.0"] [--key key.json] '
                      + '[--tamper payload|caps|keyid|sig|version]');
        process.exit(2);
    }

    const payload = fs.readFileSync(a.payload);
    const outDir = a.out;
    fs.mkdirSync(outDir, { recursive: true });

    let priv, pubRaw;
    if (a.key)
    {
        const k = JSON.parse(fs.readFileSync(a.key, 'utf8'));
        priv = await subtle.importKey('jwk', k.privateJwk,
                                      { name: 'ECDSA', namedCurve: 'P-256' }, true, ['sign']);
        pubRaw = Buffer.from(k.pubkeyRaw, 'base64');
    }
    else
    {
        const pair = await subtle.generateKey({ name: 'ECDSA', namedCurve: 'P-256' },
                                              true, ['sign', 'verify']);
        priv = pair.privateKey;
        /* raw export is 0x04 || X || Y; the schema carries X||Y only. */
        const raw = Buffer.from(await subtle.exportKey('raw', pair.publicKey));
        pubRaw = raw.subarray(1);
        fs.writeFileSync(path.join(outDir, 'key.json'), JSON.stringify({
            privateJwk: await subtle.exportKey('jwk', priv),
            pubkeyRaw: pubRaw.toString('base64'),
        }, null, 2));
    }

    const keyid = b64u((await sha256(pubRaw)).subarray(0, 8));
    const caps = a.caps ? a.caps.split(',').filter(Boolean) : [];

    const manifest = {
        skai: a.skai || '>=1.0',
        app: {
            id: a.id,
            name: a.name || a.id,
            version: a.version || '1.0.0',
        },
        publisher: {
            alg: 'ES256',
            pubkey: b64u(pubRaw),
            keyid: keyid,
        },
        capabilities: caps,
        payload: {
            type: a.type || 'js',
            size: payload.length,
            sha256: b64u(await sha256(payload)),
        },
    };

    const input = signedInput(manifest);
    const sig = Buffer.from(await subtle.sign({ name: 'ECDSA', hash: 'SHA-256' },
                                              priv, input));
    manifest.sig = b64u(sig);

    /* Tampering happens after signing on purpose: each case leaves a manifest
       that is perfectly well-formed and only fails on the property named. */
    let outPayload = payload;
    switch (a.tamper)
    {
    case undefined: case 'true': break;
    case 'payload':  outPayload = Buffer.concat([payload, Buffer.from('/*x*/')]); break;
    case 'caps':     manifest.capabilities = [...caps, 'system.settings_write']; break;
    case 'keyid':    manifest.publisher.keyid = b64u(Buffer.alloc(8, 0x5a)); break;
    case 'sig':      sig[10] ^= 0xff; manifest.sig = b64u(sig); break;
    case 'version':  manifest.app.version = '9.9.9'; break;
    default:
        console.error('unknown --tamper: ' + a.tamper);
        process.exit(2);
    }

    fs.writeFileSync(path.join(outDir, 'manifest.json'), JSON.stringify(manifest));
    fs.writeFileSync(path.join(outDir, 'payload'), outPayload);

    console.log(`${outDir}: ${manifest.app.id} keyid=${keyid} `
                + `payload=${outPayload.length}B caps=${caps.length}`
                + (a.tamper && a.tamper !== 'true' ? ` TAMPERED(${a.tamper})` : ''));
}

main().catch((e) => { console.error(e); process.exit(1); });
